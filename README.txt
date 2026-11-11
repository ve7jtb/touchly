YubiKey touch and USB power control

Prepared for <pi-user> on <pi-address>, Raspberry Pi 3 Model B Plus Rev 1.4.
OS: Trixie, 64-bit. No GPIO or USB power actions were performed during setup.

Connect with: ssh <pi-user>@<pi-address>

HARDWARE
Build the YubiPi low-capacitance NPN transistor circuit:
https://github.com/gierens/yubipi#hardware
BCM GPIO21 (physical pin 40) -> 10k resistor -> transistor base
Transistor emitter -> Pi ground; collector -> YubiKey touch pad.
Verify the actual transistor pinout before wiring. Do not connect the pad
directly to a GPIO voltage. A hardware base-to-emitter pull-down can keep
the transistor off while the Pi boots or GPIO is unconfigured.

TOUCH (after wiring)
Single touch:
  ~/touchly/touch.py once
Repeat every second, 0.5 seconds touch / 0.5 seconds release:
  ~/touchly/touch.py repeat
Stop foreground repetition with Ctrl+C.
Slower example:
  ~/touchly/touch.py repeat --interval 3 --hold 0.5

The script only drives the transistor; it does not read OTPs or forward USB
authentication to another computer. Actual touch recognition is untested.
Timing may need adjustment for the YubiKey and application.

OPTIONAL BACKGROUND TOUCH SERVICE (installed but NOT started or enabled)
  systemctl --user start yubikey-touch
  systemctl --user stop yubikey-touch
  systemctl --user status yubikey-touch
  journalctl --user -u yubikey-touch
The service is not configured to run at boot. No linger setting was changed.

POWER (all four external sockets together)
  ~/touchly/power status
  ~/touchly/power cycle
  ~/touchly/power off
  ~/touchly/power on
These commands use sudo and may request your Pi login password.
The fixed hub selector is 1-1, port 2, for the onboard hub on this Pi.
External USB devices will all disconnect, including the attached USB audio
device. Stop touch repetition before power cycling, then restart after the
YubiKey reconnects. Do not power off mounted USB storage.

NETWORK
At setup time wlan0 held <pi-address> and eth0 was down.
Plug in Ethernet, discover its assigned address, then test SSH through that
address. Actual Ethernet continuity during USB switching is untested on this
unit. The Pi 3B+ has different hub wiring from the Pi 3B.
https://github.com/mvp/uhubctl#raspberry-pi-3b

VALIDATION
GPIO tests used gpiozero's mock pin factory only. Power control was inspected
without changing power state. A YubiKey was not attached during setup.
