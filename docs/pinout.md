# Control box inputs

## Physical button (the only one)
| Button | GPIO | Function |
| --- | --- | --- |
| RC override enable | 3 | Wired between GPIO3 and GND (internal pull-up, active low, 15 ms debounce). Press toggles the joystick + RC override output. **OFF at boot.** |

Pins deliberately avoided for any future use: 0 (next to ESP_EN), **45 (reads low at boot on this board, something pulls it down)**, 7/8 (I2C to audio codec), 26/27 (second USB PHY), 34-38 (strapping/Ethernet), 36, 53/54 (audio amp and ESP32-C6 control), 28-31, 34, 35, 49-52 (Ethernet RMII). Pin choices come from the schematic text; confirm on the bench.

## Joystick buttons (Thrustmaster Solaris Base, 44 buttons, report layout in `joystick.md`)
Assigned through the `k_js_map` table in `firmware/control_box/main/buttons.c` (logical function -> joystick button bit, bit n = button n+1). Filled in from a learning session on 2026-10-01 (every press is also logged on the serial console as `joystick button N`). Joystick button 20 (bit 19) is held/latched at connect on this unit and is unused.

| Function | Joystick button | Action |
| --- | --- | --- |
| Mode 1 | 5 (bit 4) | Manual (`DO_SET_MODE`, custom mode 0) |
| Mode 2 | 6 (bit 5) | Hold (4) |
| Mode 3 | 7 (bit 6) | Auto (10) |
| Mode 4 | 8 (bit 7) | RTL (11) |
| Arm | 1 (bit 0, trigger) — needs a 0.8 s hold | `COMPONENT_ARM_DISARM` p1 = 1 |
| Disarm | 2 (bit 1) | `COMPONENT_ARM_DISARM` p1 = 0 |
| Motor on | 3 (bit 2) | RC ch6 = 2000 (only with override enabled) |
| Motor off | 4 (bit 3) | RC ch6 = 1000 |
| Trim up | 27 (bit 26) | RC ch7 = 2000 while held |
| Trim down | 26 (bit 25) | RC ch7 = 1000 while held |
| Emergency stop A + B | A = 15 (bit 14), B = 13 (bit 12) | both held 300 ms = e-stop |

Spare joystick axes (Z, Rx, Ry, Rz, Slider, Dial) and the hat remain available for future controls.

## Behaviour
- **Override enable (external button):** off at boot. Off = no `RC_CHANNELS_OVERRIDE` is sent at all (a short neutral burst is sent when disabling, then silence, so the vehicle's own RC timeout applies). Turning it on starts with motor off.
- **Emergency stop:** both stop buttons held 300 ms start a **3 s pulse** (it does not latch): neutral steering/throttle, motor off, and force-disarm (`param2 = 21196`) repeated every 200 ms until the vehicle HEARTBEAT shows disarmed; mode/arm commands are blocked during the pulse. After 3 s the controls are released again (motor stays off until motor-on is pressed) and arming works. It only fires again after both buttons have been released.
- **Motor on:** ignored unless override is enabled, no e-stop latch and the vehicle heartbeat is alive. Motor is forced off if the vehicle heartbeat is lost for 3 s.
- **Commands** (mode, arm, disarm): retried up to 3 times at 1 s intervals until `COMMAND_ACK`; the result is logged.
- **Safety gating:** a 5 s boot hold-off; any button already pressed at boot or at joystick connect counts as "held" and cannot fire until released; if the joystick disconnects, its buttons read as released.
