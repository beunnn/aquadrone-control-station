# AquaDrone GCS Hardware Controller — Full Project Plan

Derived from `AquaDrone GCS Hardware Controller — Build Plan.md`. Recommendations on open questions are marked **[Decision]**; change them if you disagree.

## 1. Goal and scope

A hardware control box (joystick + 12 buttons) that drives ArduPilot Rover vehicles via MAVLink, independent of any GCS software. A central Pi 4 running mavlink-router fans MAVLink out to the vehicle link, control devices and one or more GCS applications. A shared "selected vehicle" state keeps the Tab5, control box and Mission Planner on the same target.

**In scope:** control-box firmware, Tab5 firmware, Pi 4 hub setup, sysid-sync channel, Mission Planner sync script, RC channel mapping, failsafe behaviour, field testing.
**Out of scope (for now):** custom GCS software, video, enclosure industrial design, multi-operator arbitration.

## 1a. Confirmed requirements (from owner)

| Item | Value |
| --- | --- |
| Inputs | Thrustmaster Solaris Base (SOL-R 1) joystick (USB HID) with its 44 buttons, hat and 8 axes, plus **one external physical button** (RC override enable). No other physical buttons, dials or toggles; extra controls come from spare joystick buttons/axes |
| Buttons | 4× mode, arm, disarm, motor on, motor off, trim up, trim down, 2× emergency shutdown (both must be pressed) |
| Vehicle | ArduPilot Rover, `FRAME_CLASS` = boat (SURFACE_BOAT), separate steering + throttle, default RC config (ch1 steering, ch3 throttle) |
| Control board | Waveshare ESP32-P4-Module-DEV-KIT (with speaker) |

Verified from the Waveshare page: 100M RJ45 Ethernet (PoE header), USB 2.0 HS **Type-A host** (jumper-selectable host/device), ESP32-C6 for Wi-Fi 6/BT via SDIO, 28 free GPIOs on the 40-pin header, 8 Ω 2 W speaker header, onboard mic, MIPI DSI/CSI, TF slot, 16 MB flash, 32 MB PSRAM. **Consequences:** only one GPIO is needed (no ADC/expander; the pin-budget question is closed); Ethernet is available so use it as the primary link; the speaker gives audible feedback (arm/disarm, mode change, e-stop, link lost), which is valuable when the operator is watching the boat, not the box.

## 1b. Companion computer
The boat has a companion computer between the modem and the flight controller. It interprets RC ch6 (motor on/off) and ch7 (trim) and requires no ArduPilot firmware parameters. This is a component not shown in the original diagram; it is where the boat-side failsafe for those two channels lives.

## 2. Architecture

```
 Control box (ESP32-P4) ─┐
 Tab5 (display/selector) ─┼─ LAN/Wi-Fi ─ Pi 4: mavlink-router + broker ─ router uplink ─ Internet ─ Starlink/4G ─ AquaDrone
 Mini PC / laptop (GCS)  ─┘
```

- **Data plane:** MAVLink v2 over UDP; Pi 4 is the single endpoint for the vehicle link.
- **Control plane:** shared-state channel carrying the selected target sysid.

## 3. Key technical findings and corrections to the original plan

These are items the build plan does not cover and that will bite during implementation:

1. **ArduPilot only accepts `RC_CHANNELS_OVERRIDE` from `SYSID_MYGCS`** (default 255). The control box must send with that system ID (use a different component ID from the GCS, e.g. `MAV_COMP_ID_USER1`), or `SYSID_MYGCS` must be set accordingly. Mission Planner also uses 255; sharing a sysid is fine, but only one *override source* should be active at a time.
2. **The "target_system per vehicle" model:** several vehicles through one router means the control box sends overrides addressed to one sysid; the other vehicles must receive nothing (they will hit their own failsafe — intended). On selection change, send neutral/stop to the old vehicle first, then begin on the new one.
3. **Failsafe parameters on Rover** to set and test: `RC_OVERRIDE_TIME` (override timeout), `FS_ACTION`, `FS_GCS_ENABLE`, `FS_TIMEOUT`. Note an override of 0 / UINT16_MAX semantics: 0 = release channel, 65535 (`UINT16_MAX`) = ignore channel (MAVLink 2 extension semantics) — define behaviour per channel explicitly.
4. **Latency over Starlink/4G:** the path is controller → router → internet → modem → vehicle. Expect 50–200+ ms RTT and jitter; CGNAT on cellular/Starlink usually blocks inbound connections. Needs a VPN (WireGuard recommended; Pi 4 as a peer, vehicle companion computer or modem router as the other) or a vehicle-initiated tunnel. Do not expose raw UDP MAVLink to the internet.
5. **Security:** MAVLink has no auth by default. Enable **MAVLink 2 message signing** on the vehicle and signing keys on the control box and GCS where supported; at minimum run everything inside WireGuard.
6. **ESP32-P4 has no on-chip Wi-Fi.** Wi-Fi comes from a companion ESP32-C6 via `esp_hosted`. Prefer **Ethernet** for the control box (deterministic latency), Wi-Fi as fallback. Verify the chosen board actually exposes Ethernet/PHY and USB host.
7. **"Pico SDK" in the stack table does not apply** to an ESP32-P4 board; use ESP-IDF only.
8. **Ethernet vs Wi-Fi:** the dev kit has both; use Ethernet to the router, Wi-Fi (via C6/`esp_hosted`) only as optional fallback.
8a. **HEARTBEAT type:** send as `MAV_TYPE_GCS`, `MAV_AUTOPILOT_INVALID`.
9. **Buttons are not RC channels.** Mode, arm/disarm and e-stop are discrete commands, so they use `COMMAND_LONG` with `COMMAND_ACK` handling and confirmed state from the vehicle `HEARTBEAT` (not assumed state). Only joystick axes go in `RC_CHANNELS_OVERRIDE`. See section 5.
10. **Sol-R 1 joystick:** it is a PC gaming HID device; its report descriptor, axes (X/Y/twist/slider) and resolution are unknown to me. Capture the descriptor on Linux (`lsusb -v`, `usbhid-dump`) in Phase 0 and verify it enumerates on the P4's TinyUSB host (composite device? power draw?).

## 4. Decisions on the open questions

| Question | Decision | Rationale |
| --- | --- | --- |
| Shared selection state | **MQTT (Mosquitto on Pi 4)**, retained topic, for the Tab5 and control box. **Mission Planner is not touched:** the Pi runs a small *selection filter* that exposes one fixed GCS UDP port forwarding only the currently selected vehicle's MAVLink | Mission Planner connects once to that port and then always shows the selected vehicle, so no MP script, plugin or per-vehicle setup. This replaces the T-Encoder-style `MainV2.comPort.MAV.sysid` script, which is only needed if you later want MP to show several vehicles at once. |
| One-way vs two-way sync | **One-way** (hardware leads, GCS follows) | Falls out of the filter design; picking a vehicle in MP is no longer a thing, selection only happens on the Tab5/control box. |
| Pin budget | **Closed (revised 2026-10-01):** a single external button (override enable) on GPIO3; all other functions use joystick buttons | The joystick has 44 buttons, a hat and 8 16-bit axes; more than enough, and free axes can later carry dials (e.g. speed limit) without extra wiring. |

### Shared-state message
Topic `aquadrone/target_sysid` (retained, QoS 1), JSON: `{"sysid": 2, "src": "tab5", "seq": 41, "ts": 1760000000}`. Optional `aquadrone/ctrl/<device>/status` for presence/health.

## 5. Control mapping

### RC override (`RC_CHANNELS_OVERRIDE`, 20 Hz)
| Ch | Function | Source |
| --- | --- | --- |
| 1 | Steering | Joystick X |
| 3 | Throttle | Joystick Y (center = stop, bidirectional) |
| 6 | Motor on/off | Motor on/off buttons — latched: 1000 = off, 2000 = on |
| 7 | Motor trim | Trim up/down buttons — momentary: 1500 = inactive, 1000 = trim down (while held), 2000 = trim up (while held) |
| 10 | Throttle speed scale (used by the companion for fine speed control in harbours) | Joystick slider (descriptor Z axis, 16-bit) mapped linearly and **inverted** (owner request 2026-10-01): Z = 0 (slider bottom) = 2000 µs, Z = 65535 (top) = 1000 µs; no deadband |
| all others | `UINT16_MAX` (ignore) | — |

**Companion range rule (confirmed by owner 2026-10-01):** the boat's companion computer validates channels 1, 3, 6, 7 and 10 and requires values within 1000–2000. Steering/throttle neutral = 1500; ch6 is exactly 1000 or 2000; ch7 idle = **1500** (trim up/down = 1000/2000 only while held). The firmware clamps all of these.

1000–2000 µs, center 1500, deadband + expo, calibration stored in NVS.

**Ch 6 / ch7 rules:**
- Ch 6 defaults to 1000 (off) on boot, selection change, link loss and e-stop. It goes to 2000 only on a motor-on press, and a motor-off press returns it to 1000. Both are always sent explicitly, never ignored.
- Ch 7 is 1500 unless a trim button is held; both buttons held at once counts as 1500.
- The control box also forces neutral throttle (ch3 = 1500) while motor is off, as a second layer in case the vehicle-side function is misconfigured.
- Vehicle-side: **ch6/ch7 are interpreted by the companion computer on the boat, not by ArduPilot.** No `RCx_OPTION` or other flight-controller parameter changes are needed. The companion must receive the override stream (via the boat's MAVLink path), act on ch6/ch7, and apply the motor-off throttle gate itself. Ch6/ch7 are still harmlessly present in the override message the flight controller sees. Companion-side details (which service, which endpoint, its fail-safe if override stops) are to be confirmed; it should treat missing ch6/ch7 as motor off.
- Trim is rate-based on the vehicle, so button hold time matters: debounce at 5 ms, no auto-repeat in the box, and the display shows the last trim direction.

### Buttons (`COMMAND_LONG`, state kept in the control box)
| Button | Action |
| --- | --- |
| Mode 1–4 | `MAV_CMD_DO_SET_MODE` (custom mode). Suggested: Manual, Hold, Auto, RTL |
| Arm / Disarm | `MAV_CMD_COMPONENT_ARM_DISARM` (param1 = 1/0), wait for ACK, show result |
| Motor on / off, Trim up / down | RC channels 6 and 7 (see the table above), not commands |
| Emergency shutdown (2 buttons, both held) | Latch locally: immediately send neutral throttle, then `MAV_CMD_COMPONENT_ARM_DISARM` with param1 = 0, param2 = 21196 (force disarm), repeat until HEARTBEAT shows disarmed; speaker alarm; explicit reset required. Two separate GPIOs, both within ~200 ms and held ≥ ~300 ms to avoid accidental triggers. |

Commands go only to the currently selected vehicle (`target_system`); displayed mode/arm status comes from that vehicle's HEARTBEAT.

### Mode buttons
Manual, Hold, Auto, RTL (the ~95% set). Mapping confirmed. Further modes can be added later via a shifted button or the Tab5 UI.

## 6. Phased plan

### Phase 0 — Definition and bench setup (≈ 1 week)
- Capture Sol-R 1 HID descriptor; confirm enumeration on the P4 USB-A host.
- Answer the open mapping questions (section 5); finalize failsafe behaviour table.
- Draft button wiring/pinout (12 GPIOs + LED/speaker).
- Stand up SITL: ArduPilot Rover SITL (multiple instances, different `SYSID_THISMAV`) — all development uses SITL before the real boat.
- **Exit:** channel map doc, board confirmed, SITL running with 2 vehicles.

### Phase 1 — Pi 4 hub (≈ 3–4 days)
- Install mavlink-router as a systemd service; endpoints: vehicle link, control box, Tab5, GCS (UDP server/client as appropriate).
- Install Mosquitto; configure auth/ACLs.
- WireGuard to the vehicle network; static addressing on the LAN.
- **Exit:** QGC/Mission Planner on the laptop sees both SITL vehicles via the hub; MQTT pub/sub works.

### Phase 2 — Control box firmware v0 (≈ 2 weeks)
1. ESP-IDF project skeleton, network bring-up (Ethernet first), MAVLink C library (v2).
2. TinyUSB HID host: parse joystick report descriptor → axes/buttons. Hand-rolled generic descriptor parsing is likely needed.
3. GPIO button driver (debounce, hold and two-button chord detection), joystick filtering/deadband/calibration, speaker and LED feedback.
4. Tasks: input sampling (100 Hz) → mapper → 20 Hz `RC_CHANNELS_OVERRIDE` sender; 1 Hz `HEARTBEAT`.
5. Telemetry-in: parse `HEARTBEAT`, `SYS_STATUS`, `COMMAND_ACK`, `STATUSTEXT` for status LEDs/OLED.
6. Safety: boot in neutral; motor-on state required to send non-neutral throttle (resets to off on boot, selection change and link loss); send neutral burst on selection change or link loss.
- **Exit:** joystick drives SITL Rover via hub; unplugging cable triggers override timeout and failsafe.

### Phase 3 — Shared state and selection (≈ 1 week)
- MQTT client on control box (retained subscribe, reconnect, LWT); `target_system` follows it.
- Selection filter service on the Pi: subscribes to the MQTT topic, listens on the vehicle side of mavlink-router, and forwards only the selected `sysid` (both directions) on a fixed UDP port for the GCS. First check whether mavlink-router's endpoint options can do this (per-endpoint sysid filtering); otherwise a small pymavlink/Python proxy. When selection changes it drops the old vehicle and the GCS re-discovers the new one; test with Mission Planner and QGroundControl.
- Define behaviour if no value is retained: control box sends nothing (safe default).
- **Exit:** changing selection on any publisher moves control + MP together within <500 ms.

### Phase 4 — Tab5 firmware (≈ 2 weeks)
- ESP-IDF + M5Stack Tab5 BSP, LVGL UI: vehicle list (from observed heartbeats via hub), selected-vehicle highlight, battery/mode/arm/GPS/link-quality status.
- Publish selection; show control-box presence and link health.
- Optional: secondary joystick via USB host (reuse Phase 2 code as a shared component).
- **Exit:** tap-to-select works end to end; status accurate against SITL.

### Phase 5 — Integration, safety and field trials (≈ 2 weeks)
- Failsafe test matrix: control box power loss, Wi-Fi/Ethernet drop, Pi 4 reboot, VPN drop, vehicle modem loss, selection change while moving, two sources overriding at once.
- Latency measurement (stick → vehicle response) over Starlink/4G; tune send rate and expo/limits for latency.
- Short supervised water trials with conservative speed limits; log with vehicle dataflash and control-box log.
- **Exit:** documented test results, parameter set for the boat, go/no-go.

### Phase 6 — Hardening and docs
- Enclosure, connectors, labeling, OTA updates for ESP32 devices, config via small web page, MAVLink signing rollout, operator guide.

## 7. Repository layout (proposed)

```
aquadrone_control_station/
├── docs/                 # architecture, channel map, test matrix
├── hub/                  # mavlink-router conf, mosquitto conf, wireguard, systemd units
├── firmware/
│   ├── components/       # shared: mavlink_link, shared_state, usb_hid_joystick, input_mapper
│   ├── control_box/
│   └── tab5/
├── mp_script/            # Mission Planner sysid-follow script
├── sim/                  # SITL launch scripts, test harnesses
└── tools/                # MAVLink log/replay, RC override test sender
```

## 8. Testing strategy
- **SITL first:** 2× Rover SITL via the hub; scripted checks of override acceptance, timeout and failsafe.
- **Unit tests** (host-side) for input mapping, deadband/calibration, HID parsing against captured descriptors.
- **Hardware-in-loop bench:** control box → hub → SITL; netem (`tc`) to inject latency/jitter/loss.
- **Failsafe matrix** (Phase 5) is a release gate.

## 9. Risks

| Risk | Impact | Mitigation |
| --- | --- | --- |
| Internet latency/jitter makes driving unsafe | High | Speed limits, expo, VPN with stable route, measure early (Phase 1 with real link), consider onboard obstacle/geofence |
| HID host quirks with the chosen joystick | Med | Test the actual joystick in Phase 0; fall back to a known-simple device |
| ESP32-P4 / Tab5 software maturity | Med | Pin ESP-IDF version; prototype early |
| Accidental or missed e-stop | Med | Two-button chord, latch + audible alarm, ACK-verified repeat; vehicle-side failsafe remains the primary safety net |
| Ambiguous trim / motor semantics | Med | Resolve before Phase 2 |
| Unauthorized control | High | WireGuard + MAVLink signing, no open ports |
| Selection filter confuses Mission Planner on switch (stale params/map) | Med | Test early; fallback is a GCS reconnect or the optional MP sysid script |

## 10. Immediate next steps
1. Confirm the decisions in section 4 (MQTT, one-way sync).
2. Start Phase 0: SITL + hub on the Pi 4, then flash/test against the real Pi and Waveshare board (IPs, serial port, SSH access needed).

## 11. Progress log
- **2026-10-01 — Phase 1/2 start.** Pi mavlink-router now runs as a systemd service (`/etc/mavlink-router/main.conf`): FC via by-id serial, dev PC endpoint 192.168.2.137:14550, UDP server on :14560 for local devices. Tailscale endpoints disabled for local testing. Mosquitto not yet installed. Pi addresses are DHCP (eth0 .134, wlan0 .133); reserve them.
- **Control box bring-up verified on hardware.** `firmware/control_box` (ESP-IDF v5.5, in `/opt/esp`) gets a DHCP address over Ethernet (IP101GRI, RMII, 50 MHz external clock; MDC 31, MDIO 52, RST 51 confirmed from the Waveshare schematic), sends HEARTBEAT (sysid 255, compid 25) to the Pi :14560 and receives the vehicle (sysid 2) heartbeats. MAVLink C headers are generated locally with pymavlink into `firmware/components/mavlink_headers`. Flash port: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61097565-if00`.
- **Joystick identified.** Sol-R 1 = "Thrustmaster Solaris Base" 044f:0422; descriptor decoded and X/Y axes verified (`docs/joystick.md`).
- **USB host works on the board** (`firmware/control_box/main/joystick.c`, ESP-IDF `usb` component, no registry deps): the Solaris Base enumerates from the USB-A port with no brown-out and reports stream in. Idle values: X/Y/Rz/Ry/Rx/Dial = 32768, Z = 0, Slider = 32767; reports are sent only on change (~4/s idle), so the last value must be held and resent at 20 Hz.
- **Axis directions verified on the board** (see `docs/joystick.md`): forward = Y low (invert for throttle), left = X low; ~7% cross-talk, so use ~8–10% deadband.
- **RC override verified end to end.** `rc_task` sends `RC_CHANNELS_OVERRIDE` at 20 Hz to sysid 2 (ch1 steering, ch3 throttle inverted, 8% deadband + 0.3 expo, ch6 motor, ch7 trim, rest ignored); the flight controller reports matching `RC_CHANNELS`. Throttle gated by ch6; `CONFIG_DEV_FORCE_MOTOR_ON` (bench only, currently set in `sdkconfig.defaults`) forces ch6 = 2000 until the buttons exist. Left/right extremes and override-timeout behaviour not yet tested.
- **Steering/throttle extremes verified** (ch1/ch3 reach 1000–2000, left = 1000, forward = 2000). **Unplug test:** the box detects removal immediately, logs it and stops sending; it recovers on replug. Vehicle-side timeout NOT yet proven: `RC_CHANNELS` kept showing the last override values for 13+ s with `RC_OVERRIDE_TIME` = 3 s, probably because with no RC receiver it never reverts. Verify with `SERVO_OUTPUT_RAW` on an armed vehicle.
- **Test hygiene:** ArduDeck on the dev PC owns UDP :14550. Test scripts must not bind it (it splits packets and causes "telemetry lost"); connect them as clients of the Pi hub (`udpout:192.168.2.134:14560`) instead.
- **Owner bench test passed (armed + Manual):** servo outputs on the Giga screen follow the stick; RC failsafe works (throttle ramped from 100% to zero after unplug, normal again on replug). Vehicle-side timeout therefore confirmed.
- **Buttons implemented (13 GPIO buttons, superseded)**, pin map in `docs/pinout.md`. Bench flag removed.
- **Design change 2026-10-01:** only ONE physical button (override enable, GPIO3). All other functions move to joystick buttons through a mapping table (`k_js_map` in `main/buttons.c`). Joystick buttons already held when the stick connects are treated as held-since-connect (no events). The mapping is identified in a learning session before any command function is enabled.
- **Incident 2026-10-01:** first button firmware sent an arm command at boot because GPIO45 (originally Arm) is pulled low on the board; the test vehicle accepted it. Fixes: Arm moved to GPIO1; pins already low at boot are treated as held (no event until released); 5 s boot hold-off for all button actions; FreeRTOS tick set to 1 kHz (the 5 ms button poll starved the idle task at 100 Hz).
- **Added RC10 speed scale** from the joystick slider axis (`SPEED_SCALE_RAW` in `main.c`), clamped 1000–2000. Axis identity to be confirmed on the bench (axis-movement logging added).
- **Slider open issue (2026-10-01):** on the control box the slider produces no change in any parsed field (spare axes, vendor bytes 29..63, buttons) while the base's D1/D2 indicators blink; a Windows driver probably initialises the device (feature reports 0xF1/F2/F3 and the vendor interface 1 exist). On the dev PC the slider is js axis 2 = descriptor Z (no init needed there); RC10 now uses Z. **Verified on the control box:** the slider reports on Z after the standard HID `SET_PROTOCOL` + `SET_IDLE` requests at attach (see `docs/joystick.md`). The mapping is inverted (bottom = 2000 µs, top = 1000 µs) to match how the vehicle side interprets it. Fixed along the way: joystick state is now only valid after the first real report (a held button at connect used to look like a fresh press).
- **Joystick button mapping done** (learning session): modes 5–8, arm 1 (0.8 s hold), disarm 2, motor on/off 3/4, trim up/down 27/26, e-stop 15 + 13; see `docs/pinout.md`. **Bench-tested against sysid 2 (2026-10-01):** Manual and Hold accepted (ACK 0); Auto and RTL rejected with ACK result 4 (expected on the bench, no GPS/mission); arm (0.8 s trigger hold) and disarm accepted; emergency stop (buttons 15 + 13) latched and the vehicle ended disarmed. Motor on/off and trim only act with the override enabled, so they are untested until the GPIO3 button (or a jumper) is used. 
- **Emergency stop changed from latch to 3 s pulse (owner request, 2026-10-01):** a latch blocked re-arming after a stop on the bench. Now it forces neutral, motor off and force-disarm for 3 s, then clears; it re-fires only after both buttons are released.
- Next: wire/jumper the buttons and test each function, speaker feedback, shared vehicle selection (MQTT).
