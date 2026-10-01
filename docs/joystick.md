# Thrustmaster Solaris Base (SOL-R 1) — USB/HID findings

Captured 2026-10-01 on the dev PC. Raw descriptor: `solaris_report_descriptor.hex` (142 bytes).

## USB
- VID:PID `044f:0422`, USB 2.0 full-speed, `MaxPower` 500 mA (check the dev kit's USB-A port can supply this).
- Interface 0: HID, EP1 IN + EP1 OUT, 64 bytes, interval 4 ms. Interface 1: vendor-specific (not needed).
- Only interface 0 EP1 IN needs to be read.

## Input report (Report ID 1, 64 bytes including the ID)
| Bytes | Content |
| --- | --- |
| 0 | Report ID = 1 |
| 1–6 | 44 button bits (bit n = button n+1), then padding |
| 7–11 | padding (rest of 88 bits) |
| 12 | Hat switch (low nibble, 0–7 = 45° steps, 8+ = centered) |
| 13–28 | 8 axes, uint16 little-endian, 0–65535 (center ≈ 32768): X, Y, Z, Rz, Ry, Rx, Slider, Dial |
| 29–63 | vendor data (ignore) |

## Linux joystick mapping (js0, normalised ±32767)
| js axis | Physical | Raw descriptor axis |
| --- | --- | --- |
| 0 | Stick left/right (**steering**) | X (bytes 13–14) |
| 1 | Stick forward/back (**throttle**) | Y (bytes 15–16) |
| 2 | **Slider** (verified on the PC: moving only the slider changes only js axis 2) | Z (bytes 17–18), raw 0 at the bottom stop, 65535 at the top |
| 3, 4 | not yet identified | Rx, Ry |

Verified in isolated movement tests; sign (which direction is positive) still to be checked on the box.
Linux ordering of js axes follows evdev codes (X, Y, Z, Rx, Ry, Rz, ...), not descriptor order, so verify raw byte offsets against the board when HID host works.

## Verified on the control box (2026-10-01)
| Movement | Raw value |
| --- | --- |
| Forward | Y = 0 |
| Back | Y = 65535 |
| Left | X = 0 |
| Right | X = 65535 |
| Rest | X = Y = 32768 |

- Full 0–65535 range on both axes. Throttle must be inverted (forward = high PWM); steering maps directly.
- Mechanical cross-talk: the off axis drifts up to about ±4500 counts (~7%) at the extremes. Use a deadband of ~8–10% plus expo.

## Feature reports read from the device (GET_REPORT, read-only, 2026-10-01)
| ID | Result |
| --- | --- |
| 0xF1 | `f1 00 00 00 00 00 00 70 00 ...` (64 bytes, everything zero except byte 7 = 0x70) |
| 0xF2 | `f2 12 00 ff 01 01 20 01 00*14 14 01 06 01 21 01 00...` (looks like a table of modules/axes) |
| 0xF3 | EP0 STALL (write-only or unsupported) |

Observation: the base shows blinking `D1`/`D2` indicators next to the slider when not initialised by a host driver (Windows driver absent). The slider value does not appear in any parsed input field (axes, vendor bytes 29..63, buttons). The initialisation protocol is not documented publicly.

## Attach sequence required (resolved 2026-10-01)
Without host setup the base keeps its `D1`/`D2` indicators blinking and the slider (Z axis) reports nothing. After the control box sends the standard HID class requests `SET_PROTOCOL(report protocol)` and `SET_IDLE(0)` to interface 0 (as Linux's usbhid does), D1 lights steadily, D2 stays off, and the slider reports on **Z** (0..65535, full range verified on the control box). The vendor feature reports 0xF1/F2/F3 are not needed. Implemented in `feature_diag_task` in `firmware/control_box/main/joystick.c` (run once, 1.5 s after attach).
