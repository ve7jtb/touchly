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
