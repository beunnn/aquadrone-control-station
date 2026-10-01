# AquaDrone GCS Hardware Controller — Build Plan

Oct 1, 2026 · @Beun

A hardware joystick/dial control box decoupled from GCS software, feeding MAVLink commands through a central mavlink-router hub so any GCS (Mission Planner, QGroundControl, ArduDeck) can run mission planning and params independently of the control path.

## Architecture overview

&#91;embedded content: network topology · 6 nodes\]

The Pi 4 runs mavlink-router as the hub. The Tab5, control box, and mini PC/laptop all reach it over the local Wi-Fi network. The boat is on a separate network: its own Starlink or 4G modem meets the control station's internet connection, so the link to the Pi 4 goes through the router's internet uplink rather than a direct local link.

## Hardware components

| Device | Role | Connectivity |
| --- | --- | --- |
| Pi-shaped ESP32-P4 board (Waveshare / Viewi) | Primary control box — joystick, dials, toggles | USB-A host (joystick), GPIO/ADC (dials, toggles), Wi-Fi or Ethernet to network |
| M5Stack Tab5 + Ethernet module | Portable status display and vehicle selector; can also host a joystick | USB-A host, Wi-Fi (ESP32-C6) or Ethernet module, 5" touchscreen, battery |
| Raspberry Pi 4 | Central mavlink-router hub | Ethernet/Wi-Fi to router; endpoints to vehicle link and all control/display devices |
| Mini PC / laptop | Runs conventional GCS software (Mission Planner, QGroundControl, or ArduDeck) for mission planning, parameters, log review | Wi-Fi/Ethernet to router, one more mavlink-router endpoint |
| Wi-Fi router | Local network backbone | Connects Tab5, Pi 4, mini PC/laptop |
| AquaDrone vehicle (ArduPilot Rover) | Controlled vehicle | Telemetry link to the Pi 4 |

## MAVLink control message design

Use `RC_CHANNELS_OVERRIDE`, not `MANUAL_CONTROL`. It gives up to 18 independent channels, which maps naturally onto joystick axes plus separate dials and toggles — closer to how a real RC transmitter behaves than `MANUAL_CONTROL`'s 4 axes + button bitmask.

- Send override messages at a steady 10–20 Hz from the control box
- Send a `HEARTBEAT` at \~1 Hz from the control box's own system/component ID — ArduPilot expects to see one from a control source
- Rely on ArduPilot's RC override timeout/failsafe if the link drops, rather than assuming the last value sticks
- `target_system` on every override message is set from the current vehicle selection (see below)
- Define the RC channel mapping (which channel = which axis/dial/toggle) once the dial/toggle count is finalized

## Target system ID selection and sync

MAVLink has no protocol-level concept of a "currently selected vehicle" — a GCS's active target system ID is internal UI state only, never broadcast on the wire.

Precedent: the LilyGo T-Encoder Pro project already solves this for Mission Planner alone — the encoder is the source of truth, and a Mission Planner script (via `MainV2.comPort.MAV.sysid`, exposed to its Python/C# scripting) follows the encoder's selection.

Plan: generalize that pattern into one shared-state channel on the local network (UDP broadcast or an MQTT topic hosted on the Raspberry Pi 4) carrying the current target system ID, so every device follows the same source instead of pairwise syncing:

- Tab5 touchscreen publishes a new selection when the user taps a vehicle
- The joystick control box subscribes and updates its own `target_system` for RC overrides
- A Mission Planner script (same mechanism as the T-Encoder project) subscribes and sets `MainV2.comPort.MAV.sysid` to match

## Software and firmware stack per device

| Device | Stack |
| --- | --- |
| Pi-shaped joystick controller | ESP-IDF or Pico SDK + TinyUSB (HID host for the joystick), GPIO/ADC drivers for dials and toggles, MAVLink C library, Wi-Fi/Ethernet client publishing `RC_CHANNELS_OVERRIDE` + `HEARTBEAT`, subscriber for the shared target-sysid state |
| M5Stack Tab5 | ESP-IDF with the M5Stack Tab5 BSP, touchscreen UI for vehicle selection and status, TinyUSB HID host (if used as a secondary joystick), shared-state client |
| Raspberry Pi 4 | Linux, mavlink-router service, shared-state broker (e.g. Mosquitto if MQTT is chosen) |
| Mini PC / laptop | Mission Planner (extended with the T-Encoder-style sysid-sync script) and/or QGroundControl or ArduDeck, connected to mavlink-router as a network endpoint |

## Open questions and next steps

- [ ] Decide UDP broadcast vs MQTT for the shared target-sysid state
- [ ] Decide one-way (hardware leads, Mission Planner follows) vs two-way sysid sync
- [ ] Confirm GPIO/ADC pin budget on the chosen Pi-shaped board against the needed dial/toggle count
- [ ] Prototype TinyUSB HID host + `RC_CHANNELS_OVERRIDE` on one Pi-shaped board
- [ ] Extend the existing T-Encoder Mission Planner script to subscribe to the shared state
- [ ] Configure mavlink-router endpoints for the controller, Tab5, mini PC/laptop, and vehicle link
- [ ] Build the Tab5 touchscreen UI for vehicle selection and status
- [ ] Define the RC channel mapping (axis/dial/toggle to channel number) for the AquaDrone Rover
