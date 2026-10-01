# AquaDrone GCS hardware controller

A hardware control box (Thrustmaster Solaris Base joystick + one physical override button) on a Waveshare ESP32-P4-Module-DEV-KIT that drives ArduPilot Rover vehicles over MAVLink through a Raspberry Pi running mavlink-router. See [PLAN.md](PLAN.md) for the full plan, decisions and progress log.

## Layout
- `PLAN.md` — project plan, decisions, progress log
- `docs/` — joystick findings (`joystick.md`), input mapping and behaviour (`pinout.md`)
- `firmware/control_box/` — ESP-IDF v5.5 firmware (Ethernet, USB HID host, MAVLink)
- `firmware/components/mavlink_headers/` — generated MAVLink headers (not committed)
- `tools/gen_mavlink.sh` — regenerates the headers

## Build and flash
```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
tools/gen_mavlink.sh
# ESP-IDF v5.5 (installed in /opt/esp, alias get_idf)
get_idf
cd firmware/control_box && idf.py set-target esp32p4 && idf.py build
idf.py -p /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61097565-if00 flash
```
Hub address, port and target system ID are `menuconfig` options under "Control box" (defaults: Pi at 192.168.2.134:14560, vehicle sysid 2).

## Notes
- Test scripts must not bind UDP 14550 on the dev PC (ArduDeck owns it); connect to the hub's UDP server port 14560 instead.
- The RC override is off at boot; it is enabled by the button on GPIO3 (to GND).
