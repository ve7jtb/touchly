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
for automatic touch. Use --pcap to also save a packet capture.

TOUCH SETTINGS

Command-line flags override their corresponding environment variables.

  --pin           YUBIKEY_TOUCH_PIN           21 (BCM GPIO number)
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

The pre-charge step is a settling period. ARM leaves GPIO as a
high-impedance input with its pull resistors disabled, so the Pi does
not actively drive the touch pad while waiting. If the pin has already
been released for at least 120 ms when the first matching presence
request arrives, the daemon can skip the usual 120 ms pause before
the first sweep.

With the native GPIO backend, the first sweep runs from about 111 kHz
down toward 2 kHz and spends extra time per step in the 5.6–2 kHz
range. If another pass is needed, it starts higher, at about 500 kHz.
The daemon reports its backend in READY. The Python GPIO fallback uses
simpler toggling and does not reproduce the calibrated sweep.

USB POWER (RASPBERRY PI 3B+)

  ./power status
  ./power cycle
  ./power off
  ./power on

The power script uses sudo and uhubctl to switch all four external USB
sockets together. The on and cycle commands try to release GPIO21 before
restoring power so the YubiKey can establish an untouched baseline.
Stop touch repetition before cycling power, and unmount USB storage
before switching it off.

USB CAPTURE AND FORWARDING

Use ./pi_usbmon_capture.py --help for USB capture options, including
CSV output and optional PCAP output. See VIRTUALHERE.txt for the
separate VirtualHere USB forwarding setup.
