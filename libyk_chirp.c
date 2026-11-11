
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>

#define GPFSEL2     (0x08 / 4)
#define GPSET0      (0x1C / 4)
#define GPCLR0      (0x28 / 4)
#define GPPUD       (0x94 / 4)
#define GPPUDCLK0   (0x98 / 4)
#define CHIRP_STEPS 136

static volatile uint32_t *gpio = NULL;
static volatile int g_cancel = 0;
static volatile uint64_t g_first_edge_ns = 0;

static inline uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static inline void spin_ns(uint64_t ns) {
    uint64_t t0 = now_ns();
    while (now_ns() - t0 < ns) {
        __asm__ volatile("nop");
    }
}

// Clear the validated cancellation signal before starting a chirp pass
void yk_clear_cancel(void) {
    __atomic_store_n(&g_cancel, 0, __ATOMIC_RELEASE);
}

// Signal the active C chirp to abort and release GPIO21 after Python validates a stop condition
void yk_cancel(void) {
    __atomic_store_n(&g_cancel, 1, __ATOMIC_RELEASE);
}

// Return the software CLOCK_MONOTONIC timestamp (ns) captured in userspace immediately
// after the first `gpio[GPSET0] = mask` register write in the most recent yk_chirp_once() call
// (software GPIO-write timestamp, not an independent external physical-edge capture).
uint64_t yk_last_first_edge_ns(void) {
    return __atomic_load_n(&g_first_edge_ns, __ATOMIC_ACQUIRE);
}

int yk_init(void) {
    if (gpio && gpio != MAP_FAILED) return 0;
    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
    if (fd < 0) return -1;
    gpio = (volatile uint32_t *)mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (gpio == MAP_FAILED) {
        gpio = NULL;
        return -1;
    }
    return 0;
}

void yk_release(int pin) {
    if (pin < 0 || pin > 31) return;
    if (yk_init() < 0) return;
    uint32_t shift = (uint32_t)((pin % 10) * 3);
    uint32_t reg = (uint32_t)(pin / 10);
    uint32_t r = gpio[reg];
    r &= ~(7u << shift);
    gpio[reg] = r;

    gpio[GPPUD] = 0;
    for (volatile int i = 0; i < 150; i++) __asm__ volatile("nop");
    gpio[GPPUDCLK0] = (1u << pin);
    for (volatile int i = 0; i < 150; i++) __asm__ volatile("nop");
    gpio[GPPUD] = 0;
    gpio[GPPUDCLK0] = 0;
}

static inline void set_out(int pin) {
    uint32_t shift = (uint32_t)((pin % 10) * 3);
    uint32_t reg = (uint32_t)(pin / 10);
    uint32_t r = gpio[reg];
    r &= ~(7u << shift);
    r |=  (1u << shift);
    gpio[reg] = r;
}

// Empirical touch recognition profile measured across 16 touches on Raspberry Pi 3B+ -> YubiKey 6 NFC:
// - Steps 35..81 (half_ns = 4,500 .. 30,000 ns, 111 kHz .. 16.7 kHz) provide ~550 ms of 12 ms/step
//   pre-bias across the series coupling capacitance so the YubiKey's 385 Hz baseline tracker settles.
// - 100% of touches trigger between step 82 (half_ns = 30,658 ns, 16.31 kHz) and step 135
//   (half_ns = 250,000 ns, 2.00 kHz), with 10/10 touches in the 10-touch full-sweep benchmark
//   triggering in steps 110..135 (half_ns = 92,412 .. 250,000 ns, 5.41 kHz .. 2.00 kHz).
#define FULL_START_HALF_NS       1000
#define CALIBRATED_START_HALF_NS 4500
#define CORE_BAND_START_HALF_NS  90000
#define END_HALF_NS              250000

/**
 * Drives a logarithmic push-pull frequency chirp to trigger capacitive touch:
 * - Only aborts early when Python has validated a stop condition and set `g_cancel` via `yk_cancel()`.
 * - Returns:
 *     1 if interrupted by a validated `yk_cancel()` signal,
 *     2 if stopped by the monotonic session deadline (`rem_deadline_ms`),
 *     0 if the sweep completed within `deadline_ns`,
 *    -1 on `/dev/gpiomem` init error,
 *    -2 for a pin outside GPIO bank 0.
 */
int yk_chirp_once(int pin, int duration_ms, int rem_deadline_ms, int band) {
    if (pin < 0 || pin > 31) return -2;
    if (yk_init() < 0) return -1;
    if (duration_ms <= 0) duration_ms = 1700;
    if (rem_deadline_ms <= 0) rem_deadline_ms = duration_ms;

    int is_fallback_full = (band >= 1);
    int start_half_ns = is_fallback_full ? FULL_START_HALF_NS : CALIBRATED_START_HALF_NS;

    // Reference step dwell scaled by duration_ms (1700 ms reference -> 12 ms nominal base step)
    uint64_t base_step_ns = ((uint64_t)duration_ms * 12000000ULL) / 1700ULL;
    if (base_step_ns < 8000000ULL) {
        base_step_ns = 8000000ULL;
    }

    // Use the full hold budget (duration_ms + 250 ms) clamped to rem_deadline_ms so OS scheduling
    // jitter never truncates the critical low-frequency tail (90,000 .. 250,000 ns).
    uint64_t t_start = now_ns();
    int pulse_budget_ms = duration_ms + 250;
    int effective_ms = (rem_deadline_ms < pulse_budget_ms) ? rem_deadline_ms : pulse_budget_ms;
    uint64_t deadline_ns = t_start + (uint64_t)effective_ms * 1000000ULL;
    uint64_t session_deadline_ns = t_start + (uint64_t)rem_deadline_ms * 1000000ULL;

    uint32_t mask = (1u << pin);
    __atomic_store_n(&g_first_edge_ns, 0ULL, __ATOMIC_RELEASE);
    int recorded_first_edge = 0;
    set_out(pin);

    for (int half_ns = start_half_ns; half_ns <= END_HALF_NS; half_ns += (half_ns / 25 + 10)) {
        // Only abort when Python has validated a stop condition and set `g_cancel`
        if (__atomic_load_n(&g_cancel, __ATOMIC_ACQUIRE)) {
            yk_release(pin);
            return 1;
        }
        // Enforce monotonic deadline so the C chirp never overruns its pulse or session budget
        uint64_t now = now_ns();
        if (now >= deadline_ns) {
            yk_release(pin);
            return (now >= session_deadline_ns) ? 2 : 0;
        }

        // On Pass 0 (band == 0), boost dwell from 12 ms to 14 ms per step inside the
        // 5.56 kHz -> 2.00 kHz core trigger band (half_ns >= 90,000 ns).
        uint64_t step_ns = base_step_ns;
        if (!is_fallback_full && half_ns >= CORE_BAND_START_HALF_NS) {
            step_ns = (base_step_ns * 7ULL) / 6ULL;
        }

        int cycles = (int)(step_ns / (uint64_t)(2 * half_ns));
        if (cycles < 2) cycles = 2;
        for (int c = 0; c < cycles; c++) {
            if ((c & 15) == 0 && __atomic_load_n(&g_cancel, __ATOMIC_ACQUIRE)) {
                yk_release(pin);
                return 1;
            }
            gpio[GPSET0] = mask;
            if (!recorded_first_edge) {
                __atomic_store_n(&g_first_edge_ns, now_ns(), __ATOMIC_RELEASE);
                recorded_first_edge = 1;
            }
            spin_ns((uint64_t)half_ns);
            gpio[GPCLR0] = mask;
            spin_ns((uint64_t)half_ns);
        }
    }
    yk_release(pin);
    return 0;
}
