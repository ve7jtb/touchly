#!/usr/bin/env python3
"""Drive the YubiKey capacitive touch pad from a Raspberry Pi GPIO pin (BCM numbering).

Uses a compiled C /dev/gpiomem helper (libyk_chirp.so) to generate a smooth
logarithmic push-pull frequency chirp (500 kHz <-> 2 kHz) bounded by the caller's
explicit hold duration and session deadline, returning GPIO21 to true High-Z
input (PUD_OFF) immediately when idle or as soon as UP is satisfied.
"""
import argparse
import ctypes
import fcntl
import inspect
import math
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import RPi.GPIO as G

# Local UNIX datagram socket receiving UP_SATISFIED events from pi_usbmon_capture.py
UP_SOCK_PATH = '/tmp/yubikey_up.sock'
PEER_CREDENTIALS = struct.Struct('iII')
GPIO_MEM_PATH = '/dev/gpiomem'
SO_PATH = Path(__file__).resolve().with_name('libyk_chirp.so')
C_PATH = Path(__file__).resolve().with_name('libyk_chirp.c')
MIN_RELEASE_GAP_SEC = 0.25

C_SOURCE = r"""
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
"""

_LIB = None
_LIB_ERROR = None
_LIB_INIT_FAILED_AT = None
# Cooldown (seconds) before retrying native C compilation/initialization after a failure
LIB_RETRY_COOLDOWN_SEC = 30.0

# Native C (/dev/gpiomem) calibrated chirp waveform profile and band specs:
# - Pass 0 (band == 0): calibrated sweep 4,500..250,000 ns with 14 ms/step dwell in 90,000..250,000 ns
# - Fallback (band >= 1): repeated full sweeps 1,000..250,000 ns until UP is satisfied or session deadline expires
NATIVE_C_WAVEFORM_PROFILE = 'core_band_sweep_v2'
NATIVE_C_PASS0_SPEC = '4500..250000ns(core=90000..250000ns@14ms)'
NATIVE_C_FALLBACK_SPEC = 'band1+_repeat_until_deadline(1000..250000ns)'

# Pure-Python RPi.GPIO fallback profile: uncalibrated HIGH/LOW toggle loop without
# nanosecond frequency sweeping or multi-band differentiation, repeated until session deadline
PYTHON_FALLBACK_WAVEFORM_PROFILE = 'python_uncalibrated_toggle_v1'
PYTHON_FALLBACK_PASS0_SPEC = 'uncalibrated_gpio_toggle(no_sweep)'
PYTHON_FALLBACK_FALLBACK_SPEC = 'repeat_until_deadline(uncalibrated_gpio_toggle)'

# Backwards-compatible aliases for external callers inspecting module constants
WAVEFORM_PROFILE = NATIVE_C_WAVEFORM_PROFILE
WAVEFORM_PASS0_SPEC = NATIVE_C_PASS0_SPEC
WAVEFORM_FALLBACK_SPEC = NATIVE_C_FALLBACK_SPEC


def reset_chirp_lib_cache():
    """Clear cached native C library handle and failure cooldown so the next call retries immediately."""
    global _LIB, _LIB_ERROR, _LIB_INIT_FAILED_AT
    _LIB = None
    _LIB_ERROR = None
    _LIB_INIT_FAILED_AT = None


def get_chirp_lib(force_retry=False, allow_init=True):
    """
    Return the initialized native C library handle (`_LIB`), or `None` if unavailable.
    When `allow_init=False` (used during active session chirps, cancellation, and pin release),
    never attempt compilation, dlopen, or `yk_init()` recovery mid-session—return only the
    already-cached `_LIB` handle.
    """
    global _LIB, _LIB_ERROR, _LIB_INIT_FAILED_AT
    if _LIB is not None:
        return _LIB
    if not allow_init and not force_retry:
        return None
    # Avoid repeating compilation, dlopen, or /dev/gpiomem init on every call during fallback
    if not force_retry and _LIB_INIT_FAILED_AT is not None:
        if (time.monotonic() - _LIB_INIT_FAILED_AT) < LIB_RETRY_COOLDOWN_SEC:
            return None
    try:
        need_build = not SO_PATH.exists()
        if not need_build:
            try:
                need_build = not C_PATH.exists() or C_PATH.read_text() != C_SOURCE
            except OSError:
                need_build = False
        if need_build:
            C_PATH.write_text(C_SOURCE)
            subprocess.run(
                ['/usr/bin/gcc', '-O2', '-shared', '-fPIC', str(C_PATH), '-o', str(SO_PATH)],
                check=True,
            )
        lib = ctypes.CDLL(str(SO_PATH))
        lib.yk_init.argtypes = []
        lib.yk_init.restype = ctypes.c_int
        lib.yk_release.argtypes = [ctypes.c_int]
        lib.yk_release.restype = None
        lib.yk_clear_cancel.argtypes = []
        lib.yk_clear_cancel.restype = None
        lib.yk_cancel.argtypes = []
        lib.yk_cancel.restype = None
        if hasattr(lib, 'yk_last_first_edge_ns'):
            lib.yk_last_first_edge_ns.argtypes = []
            lib.yk_last_first_edge_ns.restype = ctypes.c_uint64
        lib.yk_chirp_once.argtypes = [
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
        ]
        lib.yk_chirp_once.restype = ctypes.c_int
        if lib.yk_init() == 0:
            _LIB = lib
            _LIB_ERROR = None
            _LIB_INIT_FAILED_AT = None
            return _LIB
        _LIB_ERROR = 'yk_init_failed(/dev/gpiomem)'
        _LIB_INIT_FAILED_AT = time.monotonic()
    except Exception as exc:
        # Record why the native C backend was unavailable and cache failure timestamp
        _LIB_ERROR = f'{type(exc).__name__}:{exc}'
        _LIB_INIT_FAILED_AT = time.monotonic()
    return None


_UNSET = object()


def get_backend_diagnostics(pin=21, hold=1.70, interval=0.45, max_duration=25.0, allow_init=True, lib_override=_UNSET):
    """Return structured backend (`native_c` vs `python_gpio`) and backend-accurate waveform-profile metadata."""
    lib = get_chirp_lib(allow_init=allow_init) if lib_override is _UNSET else lib_override
    is_native = lib is not None
    backend = 'native_c' if is_native else 'python_gpio'
    # Report the actual waveform profile and pass/fallback behavior executed by the active backend
    profile = NATIVE_C_WAVEFORM_PROFILE if is_native else PYTHON_FALLBACK_WAVEFORM_PROFILE
    pass0_spec = NATIVE_C_PASS0_SPEC if is_native else PYTHON_FALLBACK_PASS0_SPEC
    fallback_spec = NATIVE_C_FALLBACK_SPEC if is_native else PYTHON_FALLBACK_FALLBACK_SPEC
    release_gap = resolve_release_gap(hold, interval)
    info = {
        'backend': backend,
        'profile': profile,
        'pass0': pass0_spec,
        'fallback': fallback_spec,
        'pin': int(pin),
        'hold': f'{float(hold):.2f}',
        'interval': f'{float(release_gap):.2f}',
        'max_duration': f'{float(max_duration):.1f}',
    }
    if not is_native and _LIB_ERROR:
        info['fallback_reason'] = ''.join(
            ch if ch not in (' ', '\t', '\r', '\n') else '_' for ch in str(_LIB_ERROR)
        )
    return info


def select_session_backend(pin=21, hold=1.70, interval=0.45, max_duration=25.0):
    """
    Perform native-backend recovery check between sessions (`allow_init=True`) and
    return `(lib, diag)` locked for the duration of the upcoming session.
    """
    lib = get_chirp_lib(allow_init=True)
    diag = get_backend_diagnostics(
        pin=pin,
        hold=hold,
        interval=interval,
        max_duration=max_duration,
        allow_init=False,
        lib_override=lib,
    )
    return lib, diag


def format_ready_banner(pin=21, hold=1.70, interval=0.45, max_duration=25.0, diag=None):
    """Format the daemon READY line with backend and waveform-profile tokens."""
    info = diag if diag is not None else get_backend_diagnostics(pin=pin, hold=hold, interval=interval, max_duration=max_duration)
    tokens = [
        f"backend={info['backend']}",
        f"profile={info['profile']}",
        f"pass0={info['pass0']}",
        f"fallback={info['fallback']}",
        f"pin={info['pin']}",
        f"hold={info['hold']}",
        f"interval={info['interval']}",
        f"max_duration={info['max_duration']}",
    ]
    if 'fallback_reason' in info:
        tokens.append(f"fallback_reason={info['fallback_reason']}")
    return 'READY ' + ' '.join(tokens)


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be a finite positive number')
    return number


def release_pin(pin, *, strict=False):
    lib = get_chirp_lib(allow_init=False)
    if lib is not None:
        lib.yk_release(pin)
    try:
        G.setup(pin, G.IN, pull_up_down=G.PUD_OFF)
    except Exception:
        if strict:
            raise


def open_up_socket():
    """Bind a non-blocking UNIX datagram socket at UP_SOCK_PATH to receive UP_SATISFIED events."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)
        sock.setblocking(False)
        try:
            os.unlink(UP_SOCK_PATH)
        except FileNotFoundError:
            pass
        previous_umask = os.umask(0o177)
        try:
            sock.bind(UP_SOCK_PATH)
        finally:
            os.umask(previous_umask)
    except BaseException:
        sock.close()
        raise
    return sock


def make_up_state():
    """Create socket-event tracking state for `drain_up_socket`."""
    return {
        # Active USB monitor instance ID
        'current_mon': None,
        # Permanently retired USB monitor instance IDs (delayed events from these are always rejected)
        'retired_mons': set(),
        # Per-(mon, dev, cid) minimum non-retired request generation (advanced ONLY on transaction finish)
        'min_valid_gen': {},
        # Explicitly retired `(mon, dev, cid, gen)` request tuples
        'retired_gens': set(),
        # Per-(mon, dev, cid, gen) minimum valid presence-phase ID (advanced after each UP_SATISFIED phase)
        'min_valid_phase': {},
        # Per-(mon, dev, cid, gen) last assigned presence-phase ID (supports replays omitting `phase=`)
        'last_needed_phase': {},
        # Active open transaction metadata keyed by `(mon, dev, cid)` (`{'mon', 'dev', 'ep', 'cid', 'gen'}`)
        'active_tx': {},
        # Latest armed UP_NEEDED event and per-`(mon, dev, cid)` lookup table
        'latest_needed': None,
        'needed_by_chan': {},
    }


def resolve_sysfs_devnum(req_identity, up_state=None, sysfs_root='/sys/bus/usb/devices'):
    """Resolve `req_identity['dev']` from `vid`/`pid`/`serial` via sysfs when unambiguous.

    - If `up_state` already has active/needed channels matching `req_identity` with `vid`/`pid`/`serial`
      metadata, defer to `find_channel_candidate` so synthetic tests are not overridden by host sysfs.
    - Otherwise, if `/sys/bus/usb/devices` has >1 attached USB device matching `(vid, pid, serial)`,
      marks `req_identity['ambiguous'] = True`.
    - If `/sys/bus/usb/devices` has exactly 1 matching device and either `up_state` has no conflicting
      synthetic `dev` for that `cid` or that `devnum` is present in `up_state`, binds `req_identity['dev']`.
    """
    if req_identity.get('dev') is not None:
        return
    r_vid = req_identity.get('vid')
    r_pid = req_identity.get('pid')
    r_serial = req_identity.get('serial')
    if r_vid is None and r_pid is None and r_serial is None:
        return
    if up_state is not None:
        for table in (up_state.get('needed_by_chan', {}), up_state.get('active_tx', {})):
            for val in table.values():
                if any(val.get(k) is not None for k in ('vid', 'pid', 'serial')):
                    return
    base = Path(sysfs_root)
    if not base.is_dir():
        return
    matched_devs = set()
    try:
        for entry in base.iterdir():
            devnum_p = entry / 'devnum'
            vid_p = entry / 'idVendor'
            pid_p = entry / 'idProduct'
            if not (devnum_p.is_file() and vid_p.is_file() and pid_p.is_file()):
                continue
            try:
                devnum = int(devnum_p.read_text(encoding='utf-8').strip())
                vid = vid_p.read_text(encoding='utf-8').strip().lower()
                pid = pid_p.read_text(encoding='utf-8').strip().lower()
            except (OSError, ValueError):
                continue
            if r_vid is not None and vid != r_vid:
                continue
            if r_pid is not None and pid != r_pid:
                continue
            if r_serial is not None:
                serial_p = entry / 'serial'
                try:
                    serial_raw = serial_p.read_text(encoding='utf-8').strip() if serial_p.is_file() else ''
                except OSError:
                    serial_raw = ''
                serial_norm = ''.join(
                    c if c.isalnum() or c in '._:-' else '_' for c in serial_raw
                ).lower()
                if serial_norm != r_serial:
                    continue
            matched_devs.add(devnum)
    except OSError:
        return
    if len(matched_devs) > 1:
        req_identity['ambiguous'] = True
    elif len(matched_devs) == 1:
        sysfs_dev = next(iter(matched_devs))
        if up_state is not None:
            known_devs = {
                k_dev
                for table in (up_state.get('needed_by_chan', {}), up_state.get('active_tx', {}))
                for (_, k_dev, k_cid) in table.keys()
                if req_identity.get('cid') is None or k_cid == req_identity.get('cid')
            }
            if known_devs and sysfs_dev not in known_devs:
                return
        req_identity['dev'] = sysfs_dev


def matching_channel_candidates(table, req_identity, default_mon):
    """Return all `((k_mon, k_dev, k_cid), val)` entries in `table` matching `req_identity`."""
    if req_identity.get('ambiguous'):
        return []
    r_mon = req_identity.get('mon') or default_mon
    r_dev = req_identity.get('dev')
    r_cid = req_identity.get('cid')

    stage1 = []
    for (k_mon, k_dev, k_cid), val in table.items():
        if r_mon is not None and k_mon != r_mon:
            continue
        if r_dev is not None and k_dev != r_dev:
            continue
        if r_cid is not None and k_cid != r_cid:
            continue
        stage1.append(((k_mon, k_dev, k_cid), val))

    # Filter by vid/pid/serial when present on candidate entries
    for attr in ('vid', 'pid', 'serial'):
        req_val = req_identity.get(attr)
        if req_val is None or not stage1:
            continue
        if any(v.get(attr) is not None for _, v in stage1):
            stage1 = [(k, v) for k, v in stage1 if v.get(attr) == req_val]

    return stage1


def is_identity_ambiguous(state, req_identity):
    """Return True if `req_identity` matches multiple distinct `(mon, dev, cid)` channels."""
    if req_identity.get('ambiguous'):
        return True
    default_mon = state.get('current_mon')
    probe = {
        k: req_identity.get(k)
        for k in ('mon', 'dev', 'cid', 'vid', 'pid', 'serial')
        if req_identity.get(k) is not None
    }
    if req_identity.get('_unscoped_dev'):
        probe.pop('dev', None)
    keys = set()
    for table in (state['needed_by_chan'], state['active_tx']):
        for (k_mon, k_dev, k_cid), _ in matching_channel_candidates(table, probe, default_mon):
            keys.add((k_mon, k_dev, k_cid))
    return len(keys) > 1


def find_channel_candidate(table, req_identity, default_mon, state=None):
    """Return the unique matching entry in `table`, or `None` if zero or multiple channels match."""
    if state is not None and is_identity_ambiguous(state, req_identity):
        sys.stderr.write(
            f"[touch.py] Rejecting ambiguous channel lookup for {req_identity}\n"
        )
        sys.stderr.flush()
        return None
    matches = matching_channel_candidates(table, req_identity, default_mon)
    if len(matches) == 1:
        return matches[0][1]
    if len(matches) > 1:
        sys.stderr.write(
            f"[touch.py] Rejecting ambiguous channel lookup ({len(matches)} matches) for {req_identity}\n"
        )
        sys.stderr.flush()
    return None


def is_retired_gen(state, mon, dev, cid, gen):
    """Return True if `(mon, dev, cid, gen)` has been retired within monitor instance `mon`."""
    return (
        gen < state['min_valid_gen'].get((mon, dev, cid), 0)
        or (mon, dev, cid, gen) in state['retired_gens']
    )


def retire_generation(state, mon, dev, cid, gen):
    """Retire request generation `gen` on `(mon, dev, cid)` when the CTAP transaction finishes or is superseded."""
    chan_key = (mon, dev, cid)
    req_key = (mon, dev, cid, gen)
    state['retired_gens'].add(req_key)
    if gen >= state['min_valid_gen'].get(chan_key, 0):
        state['min_valid_gen'][chan_key] = gen + 1
    state['min_valid_phase'].pop(req_key, None)
    state['last_needed_phase'].pop(req_key, None)
    tx = state['active_tx'].get(chan_key)
    if tx is not None and tx['gen'] <= gen:
        state['active_tx'].pop(chan_key, None)
    cur = state['needed_by_chan'].get(chan_key)
    if cur is not None and cur['gen'] <= gen:
        state['needed_by_chan'].pop(chan_key, None)
    if (
        state['latest_needed'] is not None
        and state['latest_needed'].get('mon') == mon
        and state['latest_needed']['dev'] == dev
        and state['latest_needed']['cid'] == cid
        and state['latest_needed']['gen'] <= gen
    ):
        state['latest_needed'] = None


def parse_up_event(data):
    """Parse `UP_NEEDED|UP_SATISFIED|TX_DONE [mon=<id>] dev=<int> ep=<int> cid=<8hex> gen=<int> [phase=<int>] [final=<0|1>]`.

    Returns dict with parsed fields or `None` if malformed.
    """
    try:
        text = data.decode('ascii', errors='ignore').strip()
    except Exception:
        return None
    if not text:
        return None
    parts = text.split()
    kind = parts[0].upper()
    if kind not in ('UP_NEEDED', 'UP_SATISFIED', 'TX_DONE'):
        return None
    fields = {}
    for token in parts[1:]:
        if '=' not in token:
            continue
        k, v = token.split('=', 1)
        fields[k.lower()] = v.lower()
    try:
        # Preserve `None` when `mon=` is omitted so legacy datagrams inherit `current_mon`
        # rather than masquerading as a new monitor instance named `'default'`.
        mon = fields.get('mon')
        dev = int(fields['dev'])
        ep = int(fields['ep'])
        cid = fields['cid']
        gen = int(fields['gen'])
        phase = int(fields['phase']) if 'phase' in fields else None
        final = int(fields['final']) if 'final' in fields else 0
        usbmon_ns = int(fields['usbmon_ns']) if 'usbmon_ns' in fields else None
    except (KeyError, ValueError):
        return None
    if (mon is not None and not mon) or len(cid) != 8 or any(c not in '0123456789abcdef' for c in cid):
        return None
    vid = fields.get('vid')
    if vid is not None and (len(vid) != 4 or any(c not in '0123456789abcdef' for c in vid)):
        vid = None
    pid = fields.get('pid')
    if pid is not None and (len(pid) != 4 or any(c not in '0123456789abcdef' for c in pid)):
        pid = None
    serial = fields.get('serial') or None
    return {
        'kind': kind,
        'mon': mon,
        'dev': dev,
        'ep': ep,
        'cid': cid,
        'gen': gen,
        'phase': phase,
        'final': final,
        'usbmon_ns': usbmon_ns,
        'vid': vid,
        'pid': pid,
        'serial': serial,
    }


def parse_identity_tokens(tokens):
    """Extract optional `mon=<id>`, `dev=<int>`, `ep=<int>`, `cid=<8hex>`, `gen=<int>`, `phase=<int>`, `vid=<4hex>`, `pid=<4hex>`, `serial=<str>`, `arm_timeout=<float>`, `req_deadline=<float>` tokens from `START` / `ARM`."""
    identity = {}
    for token in tokens:
        if '=' not in token:
            continue
        k, v = token.split('=', 1)
        k = k.lower()
        v = v.lower()
        if k in ('dev', 'ep', 'gen', 'phase'):
            try:
                identity[k] = int(v)
            except ValueError:
                pass
        elif k in ('arm_timeout', 'req_deadline'):
            try:
                fval = float(v)
                if math.isfinite(fval) and fval > 0:
                    identity[f'_{k}'] = fval
            except ValueError:
                pass
        elif k == 'cid' and len(v) == 8 and all(c in '0123456789abcdef' for c in v):
            identity['cid'] = v
        elif k in ('vid', 'pid') and len(v) == 4 and all(c in '0123456789abcdef' for c in v):
            identity[k] = v
        elif k == 'serial' and v:
            identity['serial'] = v
        elif k == 'mon' and v:
            identity['mon'] = v
    return identity


def drain_up_socket(sock, active_identity=None, state=None):
    """Drain pending datagrams from `sock` and validate `(mon, dev, ep, cid, gen, phase)`.

    - Scopes generation and presence-phase tracking by monitor-instance ID (`mon`) and permanently
      retires replaced monitor instances in `state['retired_mons']`.
    - When a new monitor instance replaces `current_mon` during an active touch, invalidates
      `active_identity` and returns `'monitor_restarted'` so the caller stops the active waveform.
    - Tracks presence-phase IDs (`phase`) within each request generation (`gen`).
    - When `UP_SATISFIED` arrives with `final=0` (e.g. `UP_NEEDED -> PROCESSING`), advances only
      `min_valid_phase[(mon, dev, cid, gen)] = phase + 1` so a subsequent `UP_NEEDED` phase within
      the same request `gen` remains valid.
    - Retires `gen` ONLY when the transaction finishes (`UP_SATISFIED` with `final=1`, `TX_DONE`,
      or a newer request `gen` on the same channel).
    - Returns `'up_satisfied'`, `'transaction_finished'`, `'monitor_restarted'`, or `None`.
    """
    if sock is None:
        return None
    if state is None:
        state = make_up_state()
    matched_reason = None
    while True:
        try:
            data, ancillary, flags, _ = sock.recvmsg(
                512, socket.CMSG_SPACE(PEER_CREDENTIALS.size)
            )
            sock_rx_ns = time.monotonic_ns()
        except (BlockingIOError, InterruptedError, OSError):
            break
        if not data or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            continue
        if not any(
            level == socket.SOL_SOCKET
            and kind == socket.SCM_CREDENTIALS
            and len(credentials) == PEER_CREDENTIALS.size
            and PEER_CREDENTIALS.unpack(credentials)[1] in (0, os.geteuid())
            for level, kind, credentials in ancillary
        ):
            continue
        ev = parse_up_event(data)
        if ev is None:
            continue
        ev['sock_rx_ns'] = sock_rx_ns

        explicit_mon = ev['mon']
        mon = explicit_mon if explicit_mon is not None else (state['current_mon'] or 'default')
        ev['mon'] = mon
        dev, ep, cid, gen = ev['dev'], ev['ep'], ev['cid'], ev['gen']

        # Reject any event from a retired monitor instance so delayed old events
        # can never switch `current_mon` back or affect active/future sessions.
        if mon in state['retired_mons']:
            sys.stderr.write(
                f"[touch.py] Rejecting {ev['kind']} from retired monitor mon={mon} (current_mon={state['current_mon']})\n"
            )
            sys.stderr.flush()
            continue

        # Detect monitor replacement when a new explicit `mon` arrives (or initial monitor activation)
        if explicit_mon is not None:
            if state['current_mon'] is None:
                state['current_mon'] = mon
            elif state['current_mon'] != mon:
                old_mon = state['current_mon']
                # Retire the previous monitor instance permanently
                state['retired_mons'].add(old_mon)
                state['current_mon'] = mon
                state['active_tx'].clear()
                state['needed_by_chan'].clear()
                state['latest_needed'] = None
                # If a touch session is currently active, invalidate its binding and stop the active waveform
                if active_identity is not None:
                    active_identity.clear()
                    active_identity = None
                    if matched_reason is None:
                        matched_reason = 'monitor_restarted'

        chan_key = (mon, dev, cid)
        req_key = (mon, dev, cid, gen)

        if ev['kind'] == 'TX_DONE':
            if active_identity is None or active_identity.get('_armed_waiting_first_up'):
                # Session is idle or pre-armed (`ARM`) waiting in High-Z for the first UP_NEEDED (`phase=1`):
                # retire the completed pre-flight / prior-generation transaction without aborting the pre-arm.
                if not is_retired_gen(state, mon, dev, cid, gen):
                    retire_generation(state, mon, dev, cid, gen)
                continue

            # Reject stale TX_DONE from an already-retired generation within `mon`
            if is_retired_gen(state, mon, dev, cid, gen):
                sys.stderr.write(
                    f"[touch.py] Rejecting stale TX_DONE mon={mon} gen={gen} "
                    f"(min_valid_gen={state['min_valid_gen'].get(chan_key, 0)})\n"
                )
                sys.stderr.flush()
                continue

            # If the session identity is or has become ambiguous across multiple `(mon, dev, cid)` channels,
            # invalidate the session binding and return `'ambiguous_identity'` so the caller cancels the waveform.
            if active_identity.get('ambiguous') or is_identity_ambiguous(state, active_identity):
                sys.stderr.write(
                    f"[touch.py] Error: ambiguous session identity {active_identity} on TX_DONE {ev}; cancelling session\n"
                )
                sys.stderr.flush()
                active_identity.clear()
                active_identity = None
                if matched_reason is None:
                    matched_reason = 'ambiguous_identity'
                if not is_retired_gen(state, mon, dev, cid, gen):
                    retire_generation(state, mon, dev, cid, gen)
                continue

            # Transaction-level completion is identified by monitor instance, device, channel, and generation
            # (`mon`, `dev`, `cid`, `gen`), not endpoint (`ep`), because CTAPHID_CANCEL / replacement commands
            # arrive on the OUT endpoint (`ep=0` in usbmon TX_DONE), whereas UP_NEEDED binds `ep` to IN (`ep=5`).
            if not any(active_identity.get(k) is not None for k in ('mon', 'dev', 'cid', 'gen')):
                sys.stderr.write(f"[touch.py] Rejecting unassociated TX_DONE {ev}\n")
                sys.stderr.flush()
                continue

            # Validate `TX_DONE` against bound transaction-level fields (`mon`, `dev`, `cid`, `gen`, and optional `vid`, `pid`, `serial`)
            mismatch = False
            for k in ('mon', 'dev', 'cid', 'gen'):
                expected = active_identity.get(k)
                if expected is not None and expected != ev[k]:
                    mismatch = True
                    break
            for k in ('vid', 'pid', 'serial'):
                expected = active_identity.get(k)
                if expected is not None and ev.get(k) is not None and expected != ev.get(k):
                    mismatch = True
                    break
            if mismatch:
                sys.stderr.write(
                    f"[touch.py] Rejecting mismatched TX_DONE {ev}; expected={active_identity}\n"
                )
                sys.stderr.flush()
                continue

            # TX_DONE matched the active transaction (`mon`, `dev`, `cid`, `gen`):
            # retire its generation and stop the session with 'transaction_finished'
            retire_generation(state, mon, dev, cid, gen)
            if matched_reason is None:
                matched_reason = 'transaction_finished'
            continue

        if ev['kind'] == 'UP_NEEDED':
            # Reject UP_NEEDED from an already-retired generation within monitor instance `mon`
            if is_retired_gen(state, mon, dev, cid, gen):
                sys.stderr.write(
                    f"[touch.py] Rejecting stale UP_NEEDED mon={mon} gen={gen} "
                    f"(min_valid_gen={state['min_valid_gen'].get(chan_key, 0)})\n"
                )
                sys.stderr.flush()
                continue
            # If a newer generation arrived on the same `(mon, dev, cid)`, retire the older generation on that channel
            prev_tx = state['active_tx'].get(chan_key)
            if prev_tx is not None and prev_tx['gen'] < gen:
                retire_generation(state, mon, dev, cid, prev_tx['gen'])

            # Determine presence-phase ID within `(mon, dev, cid, gen)` (auto-incrementing if omitted by replay)
            min_phase = state['min_valid_phase'].get(req_key, 1)
            if ev['phase'] is None:
                phase = max(min_phase, state['last_needed_phase'].get(req_key, 0) + 1)
                ev['phase'] = phase
            else:
                phase = ev['phase']

            if phase < min_phase:
                sys.stderr.write(
                    f"[touch.py] Rejecting stale UP_NEEDED phase={phase} < min_phase={min_phase} for mon={mon} gen={gen}\n"
                )
                sys.stderr.flush()
                continue

            state['last_needed_phase'][req_key] = phase
            state['active_tx'].pop(chan_key, None)
            state['active_tx'][chan_key] = {
                'mon': mon,
                'dev': dev,
                'ep': ep,
                'cid': cid,
                'gen': gen,
                'vid': ev.get('vid'),
                'pid': ev.get('pid'),
                'serial': ev.get('serial'),
            }
            state['latest_needed'] = ev
            state['needed_by_chan'].pop(chan_key, None)
            state['needed_by_chan'][chan_key] = ev

            # Bind missing `('mon', 'dev', 'ep', 'cid', 'gen', 'phase')` into `active_identity`
            # only if the session identity is unambiguous across active/needed channels.
            # If ambiguity appears during an active session, invalidate `active_identity` and
            # return `'ambiguous_identity'` so `run_daemon` / `repeat` cancels the waveform immediately.
            if active_identity is not None:
                if active_identity.get('ambiguous') or is_identity_ambiguous(state, active_identity):
                    sys.stderr.write(
                        f"[touch.py] Error: session identity became ambiguous ({active_identity}) on UP_NEEDED {ev}; cancelling session\n"
                    )
                    sys.stderr.flush()
                    active_identity.clear()
                    active_identity = None
                    if matched_reason is None:
                        matched_reason = 'ambiguous_identity'
                else:
                    req_cid = active_identity.get('cid')
                    req_dev = active_identity.get('dev')
                    req_mon = active_identity.get('mon')
                    attr_ok = True
                    for k in ('vid', 'pid', 'serial'):
                        if active_identity.get(k) is not None and ev.get(k) is not None and active_identity.get(k) != ev.get(k):
                            attr_ok = False
                            break
                    if (
                        attr_ok
                        and (req_cid is None or req_cid == cid)
                        and (req_dev is None or req_dev == dev)
                        and (req_mon is None or req_mon == mon)
                    ):
                        # When pre-armed via `ARM` (`_armed_waiting_first_up`), trigger ONLY on the
                        # selected request's first UP_NEEDED (`phase == 1`) AND only if neither the
                        # pre-arm waiting deadline nor the overall request deadline has expired.
                        if active_identity.get('_armed_waiting_first_up'):
                            now_mono = time.monotonic()
                            wait_dl = active_identity.get('_arm_wait_deadline_mono')
                            req_dl = active_identity.get('_arm_req_deadline_mono')
                            if (wait_dl is not None and now_mono >= wait_dl) or (req_dl is not None and now_mono >= req_dl):
                                continue
                            if phase == 1:
                                for k in ('mon', 'dev', 'ep', 'cid', 'gen', 'phase', 'usbmon_ns', 'sock_rx_ns'):
                                    if active_identity.get(k) is None and ev.get(k) is not None:
                                        active_identity[k] = ev[k]
                                active_identity.pop('_armed_waiting_first_up', None)
                                active_identity['_first_up_triggered'] = True
                                active_identity['_first_up_mono'] = now_mono
                        else:
                            for k in ('mon', 'dev', 'ep', 'cid', 'gen', 'phase', 'usbmon_ns', 'sock_rx_ns'):
                                if active_identity.get(k) is None and ev.get(k) is not None:
                                    active_identity[k] = ev[k]
            continue

        # Event is UP_SATISFIED:
        min_phase = state['min_valid_phase'].get(req_key, 1)
        phase = ev['phase']
        if phase is None:
            # If replay omitted `phase=`, resolve to the active session's phase or last armed phase for `req_key`
            if (
                active_identity is not None
                and active_identity.get('mon', mon) == mon
                and active_identity.get('dev', dev) == dev
                and active_identity.get('cid', cid) == cid
                and active_identity.get('gen') == gen
                and active_identity.get('phase') is not None
            ):
                phase = active_identity['phase']
            else:
                phase = state['last_needed_phase'].get(req_key, min_phase)
            ev['phase'] = phase

        if active_identity is None:
            # Session is idle: advance phase floor (and retire `gen` only if `final=1`)
            if not is_retired_gen(state, mon, dev, cid, gen) and phase >= min_phase:
                state['min_valid_phase'][req_key] = phase + 1
                cur = state['needed_by_chan'].get(chan_key)
                if cur is not None and cur['gen'] == gen and cur.get('phase') == phase:
                    state['needed_by_chan'].pop(chan_key, None)
                if (
                    state['latest_needed'] is not None
                    and state['latest_needed'].get('mon') == mon
                    and state['latest_needed']['dev'] == dev
                    and state['latest_needed']['cid'] == cid
                    and state['latest_needed']['gen'] == gen
                    and state['latest_needed'].get('phase') == phase
                ):
                    state['latest_needed'] = None
                if ev['final']:
                    retire_generation(state, mon, dev, cid, gen)
            continue

        # Reject UP_SATISFIED from a retired request generation within `mon`
        if is_retired_gen(state, mon, dev, cid, gen):
            sys.stderr.write(
                f"[touch.py] Rejecting stale UP_SATISFIED mon={mon} gen={gen} "
                f"(min_valid_gen={state['min_valid_gen'].get(chan_key, 0)})\n"
            )
            sys.stderr.flush()
            continue

        # If the session identity is or has become ambiguous across multiple `(mon, dev, cid)` channels,
        # invalidate `active_identity` and return `'ambiguous_identity'` to cancel the waveform immediately.
        if active_identity.get('ambiguous') or is_identity_ambiguous(state, active_identity):
            sys.stderr.write(
                f"[touch.py] Error: ambiguous session identity {active_identity} on UP_SATISFIED {ev}; cancelling session\n"
            )
            sys.stderr.flush()
            active_identity.clear()
            active_identity = None
            if matched_reason is None:
                matched_reason = 'ambiguous_identity'
            continue

        # Require at least `cid` or `gen` to be bound on the active session
        if not any(active_identity.get(k) is not None for k in ('dev', 'ep', 'cid', 'gen')):
            sys.stderr.write(f"[touch.py] Rejecting unassociated UP_SATISFIED {ev}\n")
            sys.stderr.flush()
            continue

        # Reject if any bound field (`mon`, `dev`, 'ep', `cid`, `gen`, `phase`, or optional `vid`, `pid`, `serial`) mismatches
        mismatch = False
        for k in ('mon', 'dev', 'ep', 'cid', 'gen', 'phase'):
            expected = active_identity.get(k)
            if expected is not None and expected != ev[k]:
                mismatch = True
                break
        for k in ('vid', 'pid', 'serial'):
            expected = active_identity.get(k)
            if expected is not None and ev.get(k) is not None and expected != ev.get(k):
                mismatch = True
                break
        if mismatch:
            sys.stderr.write(
                f"[touch.py] Rejecting mismatched UP_SATISFIED {ev}; expected={active_identity}\n"
            )
            sys.stderr.flush()
            continue

        # Reject if `phase` was already satisfied earlier within the same request `gen`
        if phase < min_phase:
            sys.stderr.write(
                f"[touch.py] Rejecting stale UP_SATISFIED phase={phase} < min_phase={min_phase} for mon={mon} gen={gen}\n"
            )
            sys.stderr.flush()
            continue

        # Phase matched! Advance `min_valid_phase` for `(mon, dev, cid, gen)` so duplicate datagrams
        # for this phase are rejected, while keeping `gen` valid for any subsequent UP_NEEDED phase.
        state['min_valid_phase'][req_key] = phase + 1
        cur = state['needed_by_chan'].get(chan_key)
        if cur is not None and cur['gen'] == gen and cur.get('phase') == phase:
            state['needed_by_chan'].pop(chan_key, None)
        if (
            state['latest_needed'] is not None
            and state['latest_needed'].get('mon') == mon
            and state['latest_needed']['dev'] == dev
            and state['latest_needed']['cid'] == cid
            and state['latest_needed']['gen'] == gen
            and state['latest_needed'].get('phase') == phase
        ):
            state['latest_needed'] = None

        # Retire `gen` ONLY if this UP_SATISFIED came from a terminal response (`final=1`)
        if ev['final']:
            retire_generation(state, mon, dev, cid, gen)
        matched_reason = 'up_satisfied'
    return matched_reason


def resolve_release_gap(hold, interval):
    """Return the High-Z release gap (seconds) between consecutive chirps.

    - If `interval` is given directly as a release gap (e.g. `--interval 0.45`, where
      `interval <= hold`), use `max(MIN_RELEASE_GAP_SEC, interval)`.
    - If `interval` is given as a start-to-start cycle period (`interval > hold`, e.g.
      `--hold 1.70 --interval 2.15`), use `max(MIN_RELEASE_GAP_SEC, interval - hold)`.
    """
    gap = interval if interval <= hold else (interval - hold)
    return max(MIN_RELEASE_GAP_SEC, gap)


# Distinct chirp outcome codes returned by yk_chirp_once(), pulse_pin(), and run_validated_chirp()
CHIRP_COMPLETED = 0
CHIRP_CANCELLED = 1
CHIRP_DEADLINE = 2

# Cancellation event shared with the pure-Python GPIO fallback when libyk_chirp.so is unavailable
_PY_CANCEL = threading.Event()
# CLOCK_MONOTONIC timestamp (ns) of the first physical GPIO output write in the most recent pulse_pin() call
_LAST_PY_FIRST_EDGE_NS = 0


def get_last_first_edge_ns(lib=None):
    """Return the CLOCK_MONOTONIC timestamp (ns) of the first GPIO edge in the most recent chirp."""
    active_lib = lib if lib is not None else get_chirp_lib(allow_init=False)
    if active_lib is not None and hasattr(active_lib, 'yk_last_first_edge_ns'):
        try:
            val = int(active_lib.yk_last_first_edge_ns())
            if val > 0:
                return val
        except Exception:
            pass
    return _LAST_PY_FIRST_EDGE_NS


def clear_cancel_chirp():
    """Clear both the Python fallback cancellation event and the C cancellation flag."""
    _PY_CANCEL.clear()
    lib = get_chirp_lib(allow_init=False)
    if lib is not None:
        lib.yk_clear_cancel()


def cancel_chirp():
    """Assert both the Python fallback cancellation event and the dedicated C cancellation flag."""
    _PY_CANCEL.set()
    lib = get_chirp_lib(allow_init=False)
    if lib is not None:
        lib.yk_cancel()


def pulse_pin(pin, duration, band=0, rem_deadline=None, lib=None, use_session_lib=False):
    """
    Drive a single capacitive touch chirp for `duration` seconds using sweep `band`.
    When `use_session_lib=True`, uses the session-selected `lib` (`_LIB` or `None`) without
    re-probing or recovering `get_chirp_lib()` mid-session.
    Returns:
      - CHIRP_COMPLETED (0): sweep completed normally within the session budget
      - CHIRP_CANCELLED (1): stopped early by a validated `cancel_chirp()` signal
      - CHIRP_DEADLINE  (2): stopped because `rem_deadline` expired
    Raises:
      - RuntimeError if the native C helper returns a negative error code (e.g. -1 on /dev/gpiomem init failure)
    """
    global _LAST_PY_FIRST_EDGE_NS
    _LAST_PY_FIRST_EDGE_NS = 0
    duration_ms = max(50, int(round(duration * 1000.0)))
    rem_sec = rem_deadline if rem_deadline is not None else (duration + 1.0)
    rem_ms = max(50, int(round(rem_sec * 1000.0)))
    active_lib = lib if use_session_lib else get_chirp_lib(allow_init=True)
    if active_lib is not None:
        # Pass duration_ms, rem_deadline_ms, and band into the C chirp engine.
        # C no longer polls raw socket readability; it only aborts when Python sets g_cancel via yk_cancel().
        rc = int(active_lib.yk_chirp_once(pin, duration_ms, rem_ms, int(band)))
        if rc < 0:
            raise RuntimeError(f'yk_chirp_once failed with error code {rc}')
        if rc not in (CHIRP_COMPLETED, CHIRP_CANCELLED, CHIRP_DEADLINE):
            raise RuntimeError(f'yk_chirp_once returned unexpected status {rc}')
        return rc

    # Pure-Python fallback: honor both `_PY_CANCEL` and the remaining session budget (`rem_deadline`)
    t_start = time.monotonic()
    t_hold = t_start + duration
    t_session = (t_start + max(0.0, rem_deadline)) if rem_deadline is not None else None
    t_deadline = min(t_hold, t_session) if t_session is not None else t_hold
    recorded_first_edge = False
    G.setup(pin, G.OUT)
    try:
        while True:
            if _PY_CANCEL.is_set():
                return CHIRP_CANCELLED
            now = time.monotonic()
            if now >= t_deadline:
                if t_session is not None and now >= t_session and t_session <= t_hold:
                    return CHIRP_DEADLINE
                return CHIRP_COMPLETED
            G.output(pin, G.HIGH)
            if not recorded_first_edge:
                _LAST_PY_FIRST_EDGE_NS = time.monotonic_ns()
                recorded_first_edge = True
            G.output(pin, G.LOW)
    finally:
        release_pin(pin)


def run_validated_chirp(pin, duration, band, rem_deadline, poll_fn, lib=None, use_session_lib=False):
    """
    Run `pulse_pin` on a dedicated worker thread while the main thread calls `poll_fn`
    to read and validate `/tmp/yubikey_up.sock` datagrams and `stdin` commands.
    Unrelated metadata (`UP_NEEDED`) or rejected datagrams (mismatched `dev`/`ep`/`cid`/`gen`/`phase`,
    retired monitor, or `STOP <wrong_id>`) are consumed without interrupting the active C sweep.
    Only when `poll_fn` returns True (or raises) does Python call `cancel_chirp()`
    and join the worker before final GPIO release.
    """
    clear_cancel_chirp()
    # Preserve the distinct integer status (CHIRP_COMPLETED=0, CHIRP_CANCELLED=1, CHIRP_DEADLINE=2)
    chirp_rc = [CHIRP_COMPLETED]
    # Capture any exception raised on the worker thread so it can be re-raised on the caller thread
    worker_exc = [None]
    done_r, done_w = os.pipe()

    # Inspect poll_fn arity up front rather than catching TypeError around poll_fn(),
    # which would mask a genuine TypeError raised inside poll_fn.
    try:
        accepts_wake_fd = len(inspect.signature(poll_fn).parameters) >= 2
    except (TypeError, ValueError):
        accepts_wake_fd = False

    def _worker():
        try:
            chirp_rc[0] = pulse_pin(
                pin,
                duration,
                band=band,
                rem_deadline=rem_deadline,
                lib=lib,
                use_session_lib=use_session_lib,
            )
        except BaseException as exc:
            worker_exc[0] = exc
        finally:
            try:
                os.write(done_w, b'\x01')
            except OSError:
                pass

    t = threading.Thread(target=_worker, daemon=True)
    try:
        t.start()
        try:
            while t.is_alive():
                should_stop = poll_fn(0.50, done_r) if accepts_wake_fd else poll_fn(0.02)
                if should_stop:
                    cancel_chirp()
                    break
        finally:
            # Always assert cancellation and join the worker thread before releasing GPIO,
            # even if poll_fn raises an exception (or SystemExit / KeyboardInterrupt).
            # This prevents the worker from continuing to drive GPIO after unwinding.
            if t.is_alive():
                cancel_chirp()
            t.join()
            release_pin(pin)
        # If poll_fn completed without raising and the worker raised an exception, propagate it
        if worker_exc[0] is not None:
            raise worker_exc[0]
    finally:
        for pipe_fd in (done_r, done_w):
            try:
                os.close(pipe_fd)
            except OSError:
                pass
    return chirp_rc[0]


def run_daemon(pin, hold, interval, default_max_duration):
    """Run persistent stdin/stdout touch daemon governed by a single shared deadline per session."""
    release_pin(pin)
    up_sock = open_up_socket()
    up_fd = up_sock.fileno()

    # Select initial backend at daemon startup and emit READY with active backend and waveform-profile metadata
    session_lib, session_diag = select_session_backend(
        pin=pin, hold=hold, interval=interval, max_duration=default_max_duration
    )
    ready_line = format_ready_banner(
        pin=pin, hold=hold, interval=interval, max_duration=default_max_duration, diag=session_diag
    )
    sys.stdout.write(ready_line + '\n')
    sys.stdout.flush()
    if session_diag['backend'] != 'native_c':
        sys.stderr.write(
            f"[touch.py] WARNING: running on python_gpio fallback ({session_diag.get('fallback_reason', 'unknown')}); "
            f"profile={session_diag['profile']}\n"
        )
        sys.stderr.flush()

    fd = sys.stdin.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

    active = False
    armed_waiting_up = False
    arm_duration_sec = default_max_duration
    arm_wait_deadline_mono = 0.0
    arm_req_deadline_mono = 0.0
    arm_recovery_elapsed_sec = 0.0
    arm_released_mono = 0.0
    pre_settled_high_z = False
    in_chirp = False
    session_backend_selected = True
    pending_dones = []
    session_id = '0'
    active_identity = None
    up_state = make_up_state()
    deadline = 0.0
    band = 0
    # Per-session telemetry state (without modifying pulse timing)
    session_start_mono = 0.0
    session_pass_idx = -1
    session_passes_attempted = 0
    session_usbmon_to_edge_us = None
    session_sock_to_edge_us = None
    buf = b''
    # Compute configurable High-Z release interval between chirps (default: 0.45 s)
    release_gap = resolve_release_gap(hold, interval)

    def flush_pending_dones():
        """Emit queued DONE acknowledgements with the session-recorded backend, profile, pass index, elapsed_ms, and edge latencies."""
        while pending_dones:
            item = pending_dones.pop(0)
            done_sid, done_reason, done_meta = item
            try:
                extra_tokens = ''
                if done_meta.get('usbmon_to_edge_us') is not None:
                    extra_tokens += f" usbmon_to_edge_us={done_meta['usbmon_to_edge_us']:.1f}"
                if done_meta.get('sock_to_edge_us') is not None:
                    extra_tokens += f" sock_to_edge_us={done_meta['sock_to_edge_us']:.1f}"
                # Emit session-scoped completion with structured telemetry tokens:
                # DONE <id> <reason> backend=<backend> profile=<profile> pass=<idx> passes=<count> elapsed_ms=<ms> [usbmon_to_edge_us=<us>] [sock_to_edge_us=<us>]
                sys.stdout.write(
                    f"DONE {done_sid} {done_reason} "
                    f"backend={done_meta['backend']} profile={done_meta['profile']} "
                    f"pass={done_meta['pass_idx']} passes={done_meta['passes']} "
                    f"elapsed_ms={done_meta['elapsed_ms']:.1f}{extra_tokens}\n"
                )
                sys.stdout.flush()
            except OSError:
                pass

    def reset_arm_state():
        """Clear all ARM-specific deadlines and High-Z settling state so they never leak into a subsequent START session."""
        nonlocal armed_waiting_up, arm_duration_sec, arm_wait_deadline_mono
        nonlocal arm_req_deadline_mono, arm_recovery_elapsed_sec, arm_released_mono, pre_settled_high_z
        armed_waiting_up = False
        arm_duration_sec = default_max_duration
        arm_wait_deadline_mono = 0.0
        arm_req_deadline_mono = 0.0
        arm_recovery_elapsed_sec = 0.0
        arm_released_mono = 0.0
        pre_settled_high_z = False

    def stop_session(reason='stopped'):
        nonlocal active, active_identity
        # Signal C / Python fallback to abort any in-flight waveform now that Python has validated a stop condition
        cancel_chirp()
        if active:
            done_sid = session_id
            elapsed_ms = max(0.0, (time.monotonic() - session_start_mono) * 1000.0) if session_start_mono > 0 else 0.0
            done_meta = {
                'backend': session_diag['backend'],
                'profile': session_diag['profile'],
                'pass_idx': session_pass_idx,
                'passes': session_passes_attempted,
                'elapsed_ms': elapsed_ms,
                'usbmon_to_edge_us': session_usbmon_to_edge_us,
                'sock_to_edge_us': session_sock_to_edge_us,
            }
            active = False
            # Reset ARM-specific deadlines/flags whenever a session ends or is superseded
            reset_arm_state()
            active_identity = None
            drain_up_socket(up_sock, None, up_state)
            # Queue completion reason and telemetry; if a waveform worker is currently running (in_chirp),
            # defer both release_pin(pin) and DONE emission until run_validated_chirp() joins the worker.
            pending_dones.append((done_sid, reason, done_meta))
            if not in_chirp:
                release_pin(pin)
                flush_pending_dones()
        else:
            reset_arm_state()
            if not in_chirp:
                release_pin(pin)
            drain_up_socket(up_sock, None, up_state)

    def is_session_expired(now_mono=None):
        """Return True if the active session's current phase deadline (or pre-arm / request deadline) has expired."""
        if not active:
            return False
        t = now_mono if now_mono is not None else time.monotonic()
        if t >= deadline:
            return True
        if armed_waiting_up:
            if arm_wait_deadline_mono > 0.0 and t >= arm_wait_deadline_mono:
                return True
            if arm_req_deadline_mono > 0.0 and t >= arm_req_deadline_mono:
                return True
        return False

    def check_armed_first_up_trigger():
        """Transition a pre-armed (`ARM`) session from waiting to active Pass 0 on the first UP_NEEDED (`phase=1`)."""
        nonlocal armed_waiting_up, session_start_mono, deadline, pre_settled_high_z
        if active and armed_waiting_up and active_identity is not None and active_identity.get('_first_up_triggered'):
            first_up_mono = active_identity.pop('_first_up_mono', None) or time.monotonic()
            now_mono = time.monotonic()
            # Check arming and overall request expiry BEFORE transitioning to active touch:
            # an UP_NEEDED arriving or processed at/after the arming deadline must timeout, never extend the deadline.
            if (
                first_up_mono >= deadline
                or now_mono >= deadline
                or (arm_wait_deadline_mono > 0.0 and (first_up_mono >= arm_wait_deadline_mono or now_mono >= arm_wait_deadline_mono))
                or (arm_req_deadline_mono > 0.0 and (first_up_mono >= arm_req_deadline_mono or now_mono >= arm_req_deadline_mono))
            ):
                active_identity.pop('_first_up_triggered', None)
                stop_session('timeout')
                return True
            active_identity.pop('_first_up_triggered', None)
            armed_waiting_up = False
            session_start_mono = first_up_mono
            # Anchor the post-UP touch budget (`arm_duration_sec` minus any backend recovery time at ARM)
            # at `first_up_mono`, capped only by the overall request deadline (`arm_req_deadline_mono`).
            touch_budget_sec = max(0.0, arm_duration_sec - arm_recovery_elapsed_sec)
            deadline = min(arm_req_deadline_mono, first_up_mono + touch_budget_sec)
            if now_mono >= deadline:
                stop_session('timeout')
                return True
            # Skip the 120 ms Pass 0 pre-release sleep if GPIO21 has already been settled in High-Z for >= 120 ms
            pre_settled_high_z = (arm_released_mono > 0.0 and (first_up_mono - arm_released_mono) >= 0.12)
            return True
        return False

    def drain_commands(timeout, wake_fd=None):
        nonlocal active, armed_waiting_up, arm_duration_sec, arm_wait_deadline_mono
        nonlocal arm_req_deadline_mono, arm_recovery_elapsed_sec, arm_released_mono, pre_settled_high_z
        nonlocal session_id, active_identity, deadline, band, buf
        nonlocal session_start_mono, session_pass_idx, session_passes_attempted
        nonlocal session_usbmon_to_edge_us, session_sock_to_edge_us
        nonlocal session_lib, session_diag, session_backend_selected
        # Single shared deadline or validated local UNIX socket event (up_satisfied / transaction_finished / monitor_restarted) ends the active session
        if active:
            # 1. Check arming/session deadline expiry FIRST before draining or transitioning to active touch
            if is_session_expired():
                stop_session('timeout')
                return True
            reason = drain_up_socket(up_sock, active_identity, up_state)
            if reason:
                stop_session(reason)
                return True
            if is_session_expired():
                stop_session('timeout')
                return True
            if check_armed_first_up_trigger():
                return True
        wait_fds = [fd, up_fd] if active else [fd]
        if wake_fd is not None:
            wait_fds = wait_fds + [wake_fd]
        rlist, _, _ = select.select(wait_fds, [], [], timeout)
        if active and is_session_expired():
            stop_session('timeout')
            if fd not in rlist:
                return True
        if not rlist:
            return False
        if wake_fd is not None and wake_fd in rlist:
            try:
                os.read(wake_fd, 64)
            except OSError:
                pass
        if up_fd in rlist and active:
            if is_session_expired():
                stop_session('timeout')
                if fd not in rlist:
                    return True
            else:
                reason = drain_up_socket(up_sock, active_identity, up_state)
                if reason:
                    stop_session(reason)
                    if fd not in rlist:
                        return True
                if is_session_expired():
                    stop_session('timeout')
                    if fd not in rlist:
                        return True
                if check_armed_first_up_trigger():
                    if fd not in rlist:
                        return True
        if fd not in rlist:
            return False
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            return False
        if not chunk:
            cancel_chirp()
            if not in_chirp:
                release_pin(pin)
            sys.exit(0)
        buf += chunk
        changed = False
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            raw_line = line.decode('utf-8', errors='ignore').strip()
            if not raw_line:
                continue
            parts = raw_line.split()
            verb = parts[0].upper()
            if verb in ('START', 'ARM'):
                # Syntax:
                #   START <id> [duration_sec] [cid=<8hex>] [dev=<int>] [ep=<int>] [gen=<int>]
                #   ARM   <id> [duration_sec] [cid=<8hex>] [dev=<int>] [ep=<int>] [gen=<int>]
                # `ARM` pre-arms the session in High-Z and triggers Pass 0 at Zero-RTT (`< 1 ms`)
                # on the selected request's first `UP_NEEDED` (`phase=1`) on `/tmp/yubikey_up.sock`.
                is_prearm = (verb == 'ARM')
                start_received_mono = time.monotonic()
                new_id = session_id
                dur_sec = default_max_duration
                non_kv = [p for p in parts[1:] if '=' not in p]
                kv_tokens = [p for p in parts[1:] if '=' in p]
                if len(non_kv) >= 2:
                    new_id = non_kv[0]
                    try:
                        parsed = float(non_kv[1])
                        if math.isfinite(parsed) and parsed > 0:
                            dur_sec = parsed
                    except ValueError:
                        pass
                elif len(non_kv) == 1:
                    new_id = non_kv[0]
                start_deadline_mono = start_received_mono + dur_sec
                # If a prior session was still active, close it with reason 'superseded'
                # while `session_diag` still reflects the superseded session's backend
                if active:
                    stop_session('superseded')
                # Perform backend selection/recovery between sessions (never while in_chirp is True).
                # Any time spent in select_session_backend() is now inside start_deadline_mono (or recorded in arm_recovery_elapsed_sec).
                if not in_chirp:
                    t_rec0 = time.monotonic()
                    session_lib, session_diag = select_session_backend(
                        pin=pin, hold=hold, interval=interval, max_duration=default_max_duration
                    )
                    arm_recovery_elapsed_sec = max(0.0, time.monotonic() - t_rec0)
                    session_backend_selected = True
                    if is_prearm:
                        release_pin(pin)
                        arm_released_mono = time.monotonic()
                else:
                    arm_recovery_elapsed_sec = 0.0
                    session_backend_selected = False
                    if is_prearm:
                        arm_released_mono = 0.0
                # Drain socket before arming: records any pending UP_NEEDED and discards stale UP_SATISFIED
                drain_up_socket(up_sock, None, up_state)
                req_identity = parse_identity_tokens(kv_tokens)
                arm_timeout_opt = req_identity.pop('_arm_timeout', None)
                req_deadline_opt = req_identity.pop('_req_deadline', None)
                resolve_sysfs_devnum(req_identity, up_state)
                if 'dev' not in req_identity:
                    req_identity['_unscoped_dev'] = True
                # Bind `(mon, dev, ep, cid, gen, phase)` from the unique matching armed UP_NEEDED event
                # (or open transaction for START) if unambiguous, keyed by `(mon, dev, cid)`
                cand = None
                if req_identity.get('ambiguous') or is_identity_ambiguous(up_state, req_identity):
                    req_identity['ambiguous'] = True
                elif any(k in req_identity for k in ('mon', 'dev', 'cid', 'vid', 'pid', 'serial')):
                    cand = find_channel_candidate(
                        up_state['needed_by_chan'], req_identity, up_state['current_mon'], up_state
                    )
                    if cand is None and not is_prearm:
                        cand = find_channel_candidate(
                            up_state['active_tx'], req_identity, up_state['current_mon'], up_state
                        )
                elif up_state['latest_needed'] is not None:
                    if is_identity_ambiguous(up_state, {}):
                        req_identity['ambiguous'] = True
                    else:
                        cand = up_state['latest_needed']

                # Refuse START / ARM before driving GPIO if the target device/channel identity is ambiguous
                if req_identity.get('ambiguous'):
                    sys.stderr.write(
                        f"[touch.py] Error: refusing {verb} {new_id} due to ambiguous device/channel identity {req_identity}\n"
                    )
                    sys.stderr.flush()
                    rej_diag = (
                        session_diag
                        if session_backend_selected
                        else get_backend_diagnostics(
                            pin=pin,
                            hold=hold,
                            interval=interval,
                            max_duration=default_max_duration,
                            allow_init=False,
                        )
                    )
                    active = False
                    reset_arm_state()
                    active_identity = None
                    session_id = new_id
                    pending_dones.append((
                        new_id,
                        'ambiguous_identity',
                        {
                            'backend': rej_diag['backend'],
                            'profile': rej_diag['profile'],
                            'pass_idx': -1,
                            'passes': 0,
                            'elapsed_ms': 0.0,
                        },
                    ))
                    if not in_chirp:
                        release_pin(pin)
                        flush_pending_dones()
                    changed = True
                    continue

                if is_prearm:
                    # Separate pre-arm waiting time (`arm_wait_deadline_mono`) from the post-UP touch budget (`dur_sec`),
                    # with both capped by the overall request deadline (`arm_req_deadline_mono`).
                    arm_wait_sec = arm_timeout_opt if arm_timeout_opt is not None else max(dur_sec, 30.0)
                    req_total_sec = (
                        req_deadline_opt
                        if req_deadline_opt is not None
                        else (arm_wait_sec + dur_sec if arm_timeout_opt is not None else max(dur_sec, 55.0))
                    )
                    arm_req_deadline_mono = start_received_mono + req_total_sec
                    arm_wait_deadline_mono = min(arm_req_deadline_mono, start_received_mono + arm_wait_sec)
                    # In `ARM` mode, start immediately ONLY if the matching channel already has
                    # its first UP_NEEDED (`phase == 1`) pending in `needed_by_chan`; otherwise
                    # stay in High-Z (`armed_waiting_up = True`) until `UP_NEEDED (phase=1)` arrives.
                    cand_ready = False
                    if cand is not None:
                        cand_mon = cand.get('mon', up_state['current_mon'] or 'default')
                        if (
                            not is_retired_gen(up_state, cand_mon, cand['dev'], cand['cid'], cand['gen'])
                            and cand.get('phase', 1) == 1
                        ):
                            for k in ('mon', 'dev', 'ep', 'cid', 'gen', 'phase', 'vid', 'pid', 'serial', 'usbmon_ns', 'sock_rx_ns'):
                                if k in cand and cand[k] is not None:
                                    req_identity.setdefault(k, cand[k])
                            cand_ready = True
                    if not cand_ready:
                        req_identity['_armed_waiting_first_up'] = True
                        req_identity['_arm_wait_deadline_mono'] = arm_wait_deadline_mono
                        req_identity['_arm_req_deadline_mono'] = arm_req_deadline_mono
                    active_identity = req_identity
                    session_id = new_id
                    arm_duration_sec = dur_sec
                    session_start_mono = start_received_mono
                    session_pass_idx = -1
                    session_passes_attempted = 0
                    session_usbmon_to_edge_us = None
                    session_sock_to_edge_us = None
                    if cand_ready:
                        touch_budget_sec = max(0.0, dur_sec - arm_recovery_elapsed_sec)
                        deadline = min(arm_req_deadline_mono, time.monotonic() + touch_budget_sec)
                    else:
                        deadline = arm_wait_deadline_mono
                    active = True
                    armed_waiting_up = not cand_ready
                    pre_settled_high_z = False
                    band = 0
                    changed = True
                    continue

                if cand is not None:
                    cand_mon = cand.get('mon', up_state['current_mon'] or 'default')
                    if not is_retired_gen(up_state, cand_mon, cand['dev'], cand['cid'], cand['gen']):
                        for k in ('mon', 'dev', 'ep', 'cid', 'gen', 'phase', 'vid', 'pid', 'serial', 'usbmon_ns', 'sock_rx_ns'):
                            if k in cand and cand[k] is not None:
                                req_identity.setdefault(k, cand[k])
                # If `dev`, `cid`, `gen` are known (either from START tokens or active_tx),
                # record the open transaction keyed by `(mon, dev, cid)` and default `phase` to `min_valid_phase`.
                if not req_identity.get('ambiguous') and all(k in req_identity for k in ('dev', 'cid', 'gen')):
                    r_mon = req_identity.setdefault('mon', up_state['current_mon'] or 'default')
                    r_dev, r_cid, r_gen = req_identity['dev'], req_identity['cid'], req_identity['gen']
                    r_chan_key = (r_mon, r_dev, r_cid)
                    prev_tx = up_state['active_tx'].get(r_chan_key)
                    if prev_tx is not None and prev_tx['gen'] < r_gen:
                        retire_generation(up_state, r_mon, r_dev, r_cid, prev_tx['gen'])
                    if 'ep' in req_identity:
                        up_state['active_tx'].pop(r_chan_key, None)
                        up_state['active_tx'][r_chan_key] = {
                            'mon': r_mon,
                            'dev': r_dev,
                            'ep': req_identity['ep'],
                            'cid': r_cid,
                            'gen': r_gen,
                            'vid': req_identity.get('vid'),
                            'pid': req_identity.get('pid'),
                            'serial': req_identity.get('serial'),
                        }
                    req_identity.setdefault(
                        'phase',
                        up_state['min_valid_phase'].get((r_mon, r_dev, r_cid, r_gen), 1),
                    )
                # Accepting a host-triggered START session: explicitly clear any ARM deadlines
                # so deferred backend selection (`not session_backend_selected`) cannot cap the new
                # START deadline against a previous ARM session's `arm_req_deadline_mono`.
                reset_arm_state()
                active_identity = req_identity
                session_id = new_id
                session_start_mono = start_received_mono
                session_pass_idx = -1
                session_passes_attempted = 0
                session_usbmon_to_edge_us = None
                session_sock_to_edge_us = None
                if req_deadline_opt is not None:
                    deadline = min(start_deadline_mono, start_received_mono + req_deadline_opt)
                else:
                    deadline = start_deadline_mono
                active = True
                band = 0
                changed = True
            elif verb in ('STOP', 'RELEASE'):
                # Syntax: STOP [<id>] — only stop if <id> is omitted or matches the active session_id
                target_id = parts[1] if len(parts) > 1 else None
                if target_id is None or target_id == session_id:
                    stop_session('stopped')
                    changed = True
            elif verb in ('QUIT', 'EXIT'):
                cancel_chirp()
                if not in_chirp:
                    release_pin(pin)
                sys.exit(0)
        return changed

    while True:
        now = time.monotonic()
        # Session is controlled solely by `active` and the shared monotonic `deadline`
        if not active or now >= deadline:
            if active:
                stop_session('timeout')
            else:
                release_pin(pin)
            drain_commands(0.25)
            continue

        # If a new session was armed while a prior session's chirp worker was winding down,
        # select its backend now (between sessions, before pass 0 begins) and deduct recovery
        # time against the already-anchored `deadline`.
        if not session_backend_selected:
            t_rec0 = time.monotonic()
            session_lib, session_diag = select_session_backend(
                pin=pin, hold=hold, interval=interval, max_duration=default_max_duration
            )
            arm_recovery_elapsed_sec += max(0.0, time.monotonic() - t_rec0)
            session_backend_selected = True
            if not armed_waiting_up and arm_req_deadline_mono > 0.0:
                deadline = min(deadline, arm_req_deadline_mono)
            if time.monotonic() >= deadline:
                stop_session('timeout')
                continue

        # Mode 2 (`ARM`): keep GPIO21 settled in High-Z while waiting for the selected request's
        # first `UP_NEEDED` (`phase=1`) on `/tmp/yubikey_up.sock`.
        if armed_waiting_up:
            release_pin(pin)
            if arm_released_mono <= 0.0:
                arm_released_mono = time.monotonic()
            drain_commands(min(0.25, max(0.0, deadline - time.monotonic())))
            continue

        if band == 0:
            release_pin(pin)
            if pre_settled_high_z:
                # GPIO21 was already settled in High-Z for >= 120 ms while pre-armed (`ARM`);
                # skip the redundant 120 ms pre-release delay and start Pass 0 at Zero-RTT.
                pre_settled_high_z = False
            else:
                t_pre_end = min(deadline, time.monotonic() + 0.12)
                while time.monotonic() < t_pre_end:
                    drain_commands(min(0.02, max(0.0, t_pre_end - time.monotonic())))
                    if not active:
                        break
                if not active or time.monotonic() >= deadline:
                    continue

        # Subtract elapsed recovery + pre-release time from `deadline` before pulsing
        rem_to_deadline = deadline - time.monotonic()
        if rem_to_deadline <= 0.05:
            stop_session('timeout')
            continue

        # Snapshot the session ID, pass number (band), lib handle, and Pass 0 identity
        # BEFORE starting the worker thread. poll_during_chirp() calls drain_commands(),
        # which can process a superseding START/ARM command and mutate nonlocal
        # `session_id`, `band` (resetting it to 0), `session_lib`, and `active_identity`
        # while the worker is still running/joining.
        chirp_session_id = session_id
        chirp_band = band
        chirp_session_lib = session_lib
        pass0_identity_ref = active_identity if chirp_band == 0 else None

        def poll_during_chirp(timeout, wake_fd=None):
            drain_commands(timeout, wake_fd)
            return (not active) or (session_id != chirp_session_id)

        session_pass_idx = chirp_band
        session_passes_attempted = chirp_band + 1
        in_chirp = True
        try:
            chirp_rc = run_validated_chirp(
                pin,
                hold,
                band=chirp_band,
                rem_deadline=rem_to_deadline,
                poll_fn=poll_during_chirp,
                lib=chirp_session_lib,
                use_session_lib=True,
            )
        finally:
            # Worker thread has been cancelled and joined inside run_validated_chirp();
            # capture Pass 0 UP_NEEDED-to-first-GPIO-edge latency before flushing queued DONE events.
            in_chirp = False
            # Check the snapshotted `chirp_band` (not the live `band`, which may have been
            # reset to 0 by a superseding session during poll_during_chirp).
            if chirp_band == 0 and pass0_identity_ref is not None:
                first_edge_ns = get_last_first_edge_ns(chirp_session_lib)
                if first_edge_ns > 0:
                    u_ns = pass0_identity_ref.get('usbmon_ns')
                    s_ns = pass0_identity_ref.get('sock_rx_ns')
                    chirp_usbmon_to_edge_us = (
                        (first_edge_ns - u_ns) / 1000.0
                        if (u_ns is not None and first_edge_ns >= u_ns)
                        else None
                    )
                    chirp_sock_to_edge_us = (
                        (first_edge_ns - s_ns) / 1000.0
                        if (s_ns is not None and first_edge_ns >= s_ns)
                        else None
                    )
                    # Only update the live nonlocal session telemetry if this session is
                    # still the active session; otherwise we would contaminate a newly
                    # started/armed superseding session that has not run Pass 0 yet.
                    if session_id == chirp_session_id:
                        if chirp_usbmon_to_edge_us is not None:
                            session_usbmon_to_edge_us = chirp_usbmon_to_edge_us
                        if chirp_sock_to_edge_us is not None:
                            session_sock_to_edge_us = chirp_sock_to_edge_us
                    # Only backfill queued DONE records that belong to `chirp_session_id`
                    # so superseding sessions queued in `pending_dones` during drain_commands()
                    # do not inherit this chirp's Pass 0 edge latencies.
                    for d_sid, _, d_meta in pending_dones:
                        if d_sid != chirp_session_id:
                            continue
                        if d_meta.get('usbmon_to_edge_us') is None and chirp_usbmon_to_edge_us is not None:
                            d_meta['usbmon_to_edge_us'] = chirp_usbmon_to_edge_us
                        if d_meta.get('sock_to_edge_us') is None and chirp_sock_to_edge_us is not None:
                            d_meta['sock_to_edge_us'] = chirp_sock_to_edge_us
            release_pin(pin)
            flush_pending_dones()
        if not active or session_id != chirp_session_id:
            continue
        band += 1
        reason = drain_up_socket(up_sock, active_identity, up_state)
        if reason:
            stop_session(reason)
            continue
        if chirp_rc == CHIRP_DEADLINE or time.monotonic() >= deadline:
            stop_session('timeout')
            continue

        # Honor configurable High-Z release interval, clamped by the shared deadline
        t_end = min(deadline, time.monotonic() + release_gap)
        while active:
            left = t_end - time.monotonic()
            if left <= 0:
                break
            drain_commands(min(0.02, left))


def gpio_pin(value):
    pin = int(value)
    if not 0 <= pin <= 31:
        raise argparse.ArgumentTypeError('must be a BCM GPIO number from 0 to 31')
    return pin


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'action',
        nargs='?',
        default='touch',
        choices=(
            'touch',
            'press',
            'release',
            'repeat',
            'daemon',
            'status',
            'usb-off',
            'usb-on',
            'usb-cycle',
        ),
        help='touch or USB hub power action to run (default: touch)',
    )
    parser.add_argument(
        '--pin',
        type=gpio_pin,
        default=os.environ.get('YUBIKEY_TOUCH_PIN', '21'),
        help='BCM GPIO pin wired to the YubiKey touch contact (default: 21)',
    )
    parser.add_argument(
        '--hold',
        type=positive,
        default=os.environ.get('YUBIKEY_TOUCH_HOLD', '1.70'),
        help='seconds to hold a single 136-step chirp (default: 1.70)',
    )
    parser.add_argument(
        '--interval',
        '--release-interval',
        dest='interval',
        type=positive,
        default=os.environ.get('YUBIKEY_TOUCH_INTERVAL', '0.45'),
        help='High-Z release interval (s) between chirps, or full cycle period if > --hold (default: 0.45)',
    )
    parser.add_argument(
        '--max-duration',
        type=positive,
        default=os.environ.get('YUBIKEY_TOUCH_MAX_DURATION', '25.0'),
        help='shared wall-clock deadline (s) for repeat/daemon session (default: 25.0)',
    )
    return parser.parse_args()


def acquire_touchly_lock():
    """Acquire a single non-blocking, system-wide lock for all Touchly GPIO and USB power actions.

    Returns the open /dev/gpiomem file descriptor, which the caller must keep open
    for the lifetime of the touch session or USB hub power operation.
    """
    # Use one whole-device `flock` on `/dev/gpiomem` (`root:gpio 0660`) across all pins
    # and all USB hub power commands (`usb-off`, `usb-on`, `usb-cycle`):
    #   1. Because `uhubctl -l 1-1 -p 2` switches all four external USB ports together,
    #      a single lock prevents cycling or cutting USB power while another Touchly
    #      process is driving *any* `--pin`, regardless of UID (`jbradley` vs. `sudo`)
    #      or environment (`XDG_RUNTIME_DIR`).
    #   2. Only `root` and `gpio`-group members can open `/dev/gpiomem`, avoiding sticky
    #      directory `fs.protected_regular=2` cross-UID errors and unprivileged lock squatting.
    #   3. `flock` is bound to `lock_fd`'s open file description, so `libyk_chirp`'s
    #      `yk_init()` opening and closing its own `/dev/gpiomem` fd does not drop the lock,
    #      and `O_CLOEXEC` avoids leaking `/dev/gpiomem` into the `sudo uhubctl` child.
    try:
        lock_fd = os.open(GPIO_MEM_PATH, os.O_RDWR | os.O_CLOEXEC)
    except OSError as exc:
        sys.exit(f'Cannot open {GPIO_MEM_PATH} to acquire Touchly hardware lock: {exc}')

    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(lock_fd)
        sys.exit('Touchly GPIO/USB hardware is busy. Stop the active touch session or daemon first.')
    except BaseException:
        os.close(lock_fd)
        raise
    return lock_fd


def main():
    args = parse_args()

    # Keep `lock_fd` bound in `main()`'s frame so the single Touchly lock remains held
    # until the process exits (including across `subprocess.run(...)` in `usb-*` actions).
    lock_fd = acquire_touchly_lock()

    G.setwarnings(False)
    G.setmode(G.BCM)

    def handle_signal(_signum, _frame):
        try:
            cancel_chirp()
            release_pin(args.pin)
        finally:
            sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGHUP, handle_signal)
    signal.signal(signal.SIGPIPE, handle_signal)

    if args.action == 'daemon':
        try:
            run_daemon(args.pin, args.hold, args.interval, args.max_duration)
        finally:
            release_pin(args.pin)
        return

    if args.action in ('release', 'usb-off', 'usb-on', 'usb-cycle'):
        # Drive the configured touch pin to High-Z before switching USB VBUS off/on/cycle
        # so an unpowered key is never back-driven and power-up sees an untouched baseline.
        release_pin(args.pin, strict=True)
        if args.action == 'release':
            return
        command = ['sudo', '/usr/sbin/uhubctl', '-l', '1-1', '-p', '2']
        if args.action == 'usb-off':
            command += ['-a', 'off']
        elif args.action == 'usb-on':
            command += ['-a', 'on']
        else:
            command += ['-a', 'cycle', '-d', '2']
        # Keep the Touchly lock while USB power switches and the key starts up.
        sys.exit(subprocess.run(command).returncode)

    if args.action == 'press':
        pulse_pin(args.pin, args.hold, band=0)
        return

    if args.action == 'status':
        release_pin(args.pin)
        diag = get_backend_diagnostics(
            pin=args.pin, hold=args.hold, interval=args.interval, max_duration=args.max_duration
        )
        extra = f" fallback_reason={diag['fallback_reason']}" if 'fallback_reason' in diag else ''
        print(
            f"BCM pin {args.pin}: released (High-Z input, PUD_OFF) "
            f"backend={diag['backend']} profile={diag['profile']} "
            f"pass0={diag['pass0']} fallback={diag['fallback']} "
            f"hold={diag['hold']}s interval={diag['interval']}s{extra}"
        )
        return

    try:
        if args.action == 'touch':
            release_pin(args.pin)
            time.sleep(0.12)
            pulse_pin(args.pin, args.hold, band=0)
            release_pin(args.pin)
            return

        # Repeat mode: run chirps separated by `release_gap` until the shared monotonic
        # `deadline` expires or UP is satisfied / transaction finishes on the local UNIX socket.
        up_sock = open_up_socket()
        up_fd = up_sock.fileno()
        repeat_state = make_up_state()
        drain_up_socket(up_sock, None, repeat_state)
        if is_identity_ambiguous(repeat_state, {}):
            sys.stderr.write('[touch.py] Error: refusing repeat touch due to ambiguous device/channel state\n')
            sys.stderr.flush()
            release_pin(args.pin)
            sys.exit(1)
        repeat_identity = dict(repeat_state['latest_needed']) if repeat_state['latest_needed'] else {'_unscoped_dev': True}
        repeat_identity.pop('kind', None)
        repeat_identity.pop('final', None)
        release_pin(args.pin)
        release_gap = resolve_release_gap(args.hold, args.interval)
        deadline = time.monotonic() + args.max_duration
        # Poll socket during the initial 120 ms pre-release window so an early completion exits immediately
        t_pre_end = min(deadline, time.monotonic() + 0.12)
        while time.monotonic() < t_pre_end:
            rlist, _, _ = select.select([up_fd], [], [], min(0.02, max(0.0, t_pre_end - time.monotonic())))
            if rlist and drain_up_socket(up_sock, repeat_identity, repeat_state):
                return
        band = 0
        repeat_done = False

        def poll_repeat(timeout, wake_fd=None):
            nonlocal repeat_done
            if time.monotonic() >= deadline:
                return True
            wait_fds = [up_fd, wake_fd] if wake_fd is not None else [up_fd]
            rlist, _, _ = select.select(wait_fds, [], [], timeout)
            if wake_fd is not None and wake_fd in rlist:
                try:
                    os.read(wake_fd, 64)
                except OSError:
                    pass
            if up_fd in rlist and drain_up_socket(up_sock, repeat_identity, repeat_state):
                repeat_done = True
                return True
            return False

        while time.monotonic() < deadline:
            if repeat_done or drain_up_socket(up_sock, repeat_identity, repeat_state):
                return
            rem_to_deadline = deadline - time.monotonic()
            if rem_to_deadline <= 0.05:
                break
            chirp_rc = run_validated_chirp(
                args.pin,
                args.hold,
                band=band,
                rem_deadline=rem_to_deadline,
                poll_fn=poll_repeat,
            )
            release_pin(args.pin)
            band += 1
            if repeat_done or drain_up_socket(up_sock, repeat_identity, repeat_state) or chirp_rc == CHIRP_DEADLINE or time.monotonic() >= deadline:
                return
            t_end = min(deadline, time.monotonic() + release_gap)
            while time.monotonic() < t_end:
                rlist, _, _ = select.select([up_fd], [], [], min(0.02, max(0.0, t_end - time.monotonic())))
                # Exit repeat mode immediately (`return` runs `finally: release_pin(args.pin)`)
                # rather than `break`, which only exited the inner release-gap loop after consuming the event.
                if rlist and drain_up_socket(up_sock, repeat_identity, repeat_state):
                    return
    finally:
        release_pin(args.pin)


if __name__ == '__main__':
    main()
