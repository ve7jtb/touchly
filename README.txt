Touchly: YubiKey touch and USB power control

Touchly uses a Raspberry Pi GPIO pin to trigger the capacitive touch
sensor on a USB-connected YubiKey. It also includes tools for capturing
USB traffic and cycling power to the Raspberry Pi 3B+ USB ports.

HARDWARE

Plug the YubiKey into the Raspberry Pi. Connect a wire from BCM GPIO21
(physical pin 40) directly to the YubiKey’s capacitive touch pad. Use a
small piece of copper tape to hold the wire against the pad.

During a touch, touch.py drives the pin with a frequency sweep. Between
touches, it leaves the pin as a high-impedance input with its internal
pull resistors disabled.

FRESH SETUP (RASPBERRY PI 3B+, RASPBERRY PI OS TRIXIE)

On the Pi, install Python, RPi.GPIO, and the compiler and headers for
the native GPIO helper. Git is used to clone the project, and usbutils
provides lsusb for finding the YubiKey.

  sudo apt update
  sudo apt install git python3 python3-rpi.gpio gcc libc6-dev usbutils
  git clone https://github.com/ve7jtb/touchly.git
  cd touchly

If you also want to use the USB power script, install uhubctl:

  sudo apt install uhubctl

Run touch.py as your normal user. If it cannot access /dev/gpiomem,
check that your account is in the gpio group. If you need to add it,
run the following as your normal login user, then log out and back in:

  sudo usermod -aG gpio "$USER"

VERIFY AUTOMATIC TOUCH

1. Plug in the YubiKey and connect the GPIO wire as described above.
   Run lsusb on the Pi and note the YubiKey's Bus and Device numbers.

2. In one Pi terminal, start the touch daemon from the project directory:

     ./touch.py daemon

   Wait for READY with backend=native_c. If it reports
   backend=python_gpio, check the warning for a compiler or GPIO access
   problem before testing the native sweep.

3. In a second Pi terminal, start the monitor on the YubiKey's USB bus.
   These commands assume Bus 001. Change --bus to match lsusb.

     sudo modprobe usbmon
     sudo ./pi_usbmon_capture.py --bus 1

   If the YubiKey's Device number is below 5, add --min-devnum with
   that number. For example, use --min-devnum 4 for Device 004.

4. Have a FIDO operation ready on the computer that can access the
   YubiKey. For example, use a sign-in that already has this key
   registered. In the daemon terminal, enter:

     ARM setup-check

   Within 30 seconds, start the operation that normally requires a
   touch. Keep your hands off the YubiKey. If the operation succeeds
   and the daemon reports DONE setup-check with passes=1 or higher,
   automatic touch worked. A reason such as up_satisfied or
   transaction_finished shows how the daemon ended the session.

   If DONE reports timeout with passes=0, check the USB bus, the
   device-number cutoff, and whether the operation actually requested
   a touch. If passes is greater than zero but the prompt remains,
   check the wire and copper tape contact. The pass count shows that
   a sweep was attempted.

5. Enter QUIT in the daemon terminal and press Ctrl+C in the monitor
   terminal when finished.

TOUCH

Run these commands from the project directory:

  ./touch.py touch       # Trigger one touch, then release the pin
  ./touch.py repeat      # Repeat touches for up to 25 seconds
  ./touch.py release     # Set the pin to high-impedance input
  ./touch.py --help      # Show available commands and options

The default touch sweep lasts 1.70 seconds, with a 0.45-second release
between sweeps in repeat mode. Use --hold and --interval to adjust the
timing, and --max-duration to set the repeat deadline. Press Ctrl+C to
stop repeat mode early.

touch.py uses RPi.GPIO. It can build its native GPIO helper with GCC.
The USB capture script can report when a FIDO request's user-presence
check has been satisfied so repeat mode can stop early.

AUTOMATIC TOUCH

Start the touch daemon in one terminal:

  ./touch.py daemon

Wait for READY. In a second terminal, start the USB monitor on the Pi:

  sudo ./pi_usbmon_capture.py --bus 1

In the daemon terminal, enter the following line before starting a FIDO
operation that will ask for a touch:

  ARM login

The daemon keeps GPIO released until the USB monitor reports the first
matching request for user presence (UP_NEEDED). It then starts the touch
sweep and can stop early when the monitor reports that the presence
phase ended or the transaction finished. The daemon prints DONE with
the request ID and the reason it stopped. Send a new ARM for each
operation. A host program can send the same lines to the daemon's stdin
and read READY and DONE from stdout.

  ARM login 10 arm_timeout=15 req_deadline=20
  START login 10
  STOP login
  QUIT

ARM waits for a presence request. START begins a touch session without
waiting for one. STOP releases the pin for the named session, and QUIT
exits the daemon. In the ARM example, the daemon waits up to 15 seconds
for the request, allows up to 10 seconds for touching, and stops within
20 seconds total from ARM. With the standard defaults, bare ARM allows
30 seconds of waiting, 25 seconds of touching, and 55 seconds total.
START and ARM can also take device and channel selectors, such as
dev=<USB-device-number> and cid=<8-hex-digit-channel-ID>, to target a
specific request when several devices or channels are active.

The USB monitor reads /dev/usbmon1 by default. Use --bus if the YubiKey
is on another USB bus, and lower --min-devnum if its device number is
below the default cutoff of 5. The default HID capture mode is enough
for automatic touch. Use --pcap to also save a packet capture. The
touch daemon accepts events from the same user or from root, which
allows the USB monitor to run with sudo.

TOUCH SETTINGS

Command-line flags override their corresponding environment variables.

  --pin           YUBIKEY_TOUCH_PIN           21 (BCM GPIO 0–31)
  --hold          YUBIKEY_TOUCH_HOLD          1.70 seconds (sweep timing)
  --interval      YUBIKEY_TOUCH_INTERVAL      0.45 seconds
  --max-duration  YUBIKEY_TOUCH_MAX_DURATION  25 seconds

--release-interval is an alias for --interval. If the interval is at
most --hold, it sets the release gap. If it is greater than --hold, it
sets the nominal start-to-start period. The release gap is always at
least 0.25 seconds. --max-duration sets the repeat deadline and the
default touch budget for daemon sessions. A duration on START or ARM
overrides that session's default touch budget.

PRE-ARMING AND THE FIRST SWEEP

Before the first sweep, the daemon normally gives the pad 120 ms to
settle. During this time GPIO is a high-impedance input with its pull
resistors disabled. If ARM has already kept the pin released for at
least 120 ms when the matching presence request arrives, the daemon
can skip that pause.

The active pre-bias begins after that settling period, as part of the
first native sweep. The Pi drives the pad from about 111 kHz down to
16.7 kHz during this phase. At default timing this takes roughly 550 ms
on the Pi 3B+ and YubiKey 6 NFC setup used to tune the waveform. The
sweep then continues toward 2 kHz, spending extra time per step in the
5.6–2 kHz range. Later passes start at about 500 kHz.
The daemon reports its backend in READY. The Python GPIO fallback uses
simpler toggling and does not reproduce the calibrated sweep.

USB POWER (RASPBERRY PI 3B+)

  ./power status
  ./power cycle
  ./power off
  ./power on

The power script uses sudo and uhubctl to switch all four external USB
sockets together. Stop the touch daemon or repeat mode before running
on or cycle. These commands require GPIO release and hold the pin lock
while restoring power so the YubiKey can establish an untouched
baseline. Unmount USB storage before switching it off.

USB CAPTURE AND FORWARDING

Use ./pi_usbmon_capture.py --help for USB capture options, including
CSV output and optional PCAP output. By default, each run creates a
private directory under /tmp and prints its capture output prefix.
For background capture, the prefix is written to /run/pi_usbmon_capture.log.
If you specify --output-prefix or --pidfile, choose paths that do not
already exist. See VIRTUALHERE.txt for the separate VirtualHere USB
forwarding setup.
