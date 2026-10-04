"""Emulate extra boats on the hub so vehicle selection can be tested with one real flight controller.

Each fake sysid sends HEARTBEAT and SYS_STATUS at 1 Hz, ACKs arm/disarm/mode commands and logs the
RC overrides addressed to it (including when the stream starts and stops).
Connects as a client of the Pi hub; never bind :14550 on the dev PC (ArduDeck owns it).

    python tools/fake_vehicles.py --hub 192.168.2.134:14562 --sysids 3,4
"""
import argparse
import time

from pymavlink import mavutil

M = mavutil.mavlink
ROVER_MODES = {0: "MANUAL", 4: "HOLD", 10: "AUTO", 11: "RTL"}
OVERRIDE_GAP_S = 0.5


def fmt_ovr(m):
    return f"ch1 {m.chan1_raw} ch3 {m.chan3_raw} ch6 {m.chan6_raw} ch7 {m.chan7_raw} ch10 {m.chan10_raw}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hub", default="192.168.2.134:14562")
    ap.add_argument("--sysids", default="3,4")
    args = ap.parse_args()
    ids = [int(s) for s in args.sysids.split(",")]

    conn = mavutil.mavlink_connection(f"udpout:{args.hub}", source_system=ids[0], source_component=1)
    veh = {i: {"armed": False, "mode": 4, "last_ovr": 0.0, "last_log": 0.0} for i in ids}
    next_hb = 0.0

    while True:
        now = time.time()
        if now >= next_hb:
            next_hb = now + 1.0
            for i, v in veh.items():
                conn.mav.srcSystem = i
                base = M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (M.MAV_MODE_FLAG_SAFETY_ARMED if v["armed"] else 0)
                conn.mav.heartbeat_send(M.MAV_TYPE_SURFACE_BOAT, M.MAV_AUTOPILOT_ARDUPILOTMEGA, base, v["mode"], M.MAV_STATE_ACTIVE)
                conn.mav.sys_status_send(0, 0, 0, 500, 12600 + 100 * i, -1, 80 - i, 0, 0, 0, 0, 0, 0)
                if v["last_ovr"] and now - v["last_ovr"] > OVERRIDE_GAP_S:
                    print(f"[{i}] override stream stopped, last {fmt_ovr(v['last_msg'])}")
                    v["last_ovr"] = 0.0

        msg = conn.recv_match(blocking=True, timeout=0.05)
        if msg is None:
            continue
        t = msg.get_type()
        target = getattr(msg, "target_system", None)
        if target not in veh:
            continue
        v = veh[target]
        conn.mav.srcSystem = target

        if t == "COMMAND_LONG":
            result = M.MAV_RESULT_ACCEPTED
            if msg.command == M.MAV_CMD_COMPONENT_ARM_DISARM:
                v["armed"] = msg.param1 == 1
                desc = "ARM" if v["armed"] else ("FORCE DISARM" if int(msg.param2) == 21196 else "DISARM")
            elif msg.command == M.MAV_CMD_DO_SET_MODE:
                v["mode"] = int(msg.param2)
                desc = f"mode {ROVER_MODES.get(v['mode'], v['mode'])}"
            else:
                result = M.MAV_RESULT_UNSUPPORTED
                desc = f"command {msg.command}"
            print(f"[{target}] {desc} from {msg.get_srcSystem()}/{msg.get_srcComponent()} -> ACK {result}")
            conn.mav.command_ack_send(msg.command, result, 0, 0, msg.get_srcSystem(), msg.get_srcComponent())

        elif t == "RC_CHANNELS_OVERRIDE":
            if not v["last_ovr"]:
                print(f"[{target}] override stream started")
            v["last_ovr"] = time.time()
            if v["last_ovr"] - v["last_log"] >= 1.0:
                v["last_log"] = v["last_ovr"]
                print(f"[{target}] ovr {fmt_ovr(msg)}")
            v["last_msg"] = msg


if __name__ == "__main__":
    main()
