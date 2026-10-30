#!/usr/bin/env python3
"""Quick survey of a MAVLink .tlog: message counts, sources, modes, param history."""
import sys
from collections import Counter, defaultdict

from pymavlink import mavutil

path = sys.argv[1]
m = mavutil.mavlink_connection(path, dialect="ardupilotmega", robust_parsing=True)

counts = Counter()
src = Counter()
first_t = last_t = None
modes = []  # (t, mode_str, armed)
last_mode = None
params = defaultdict(list)  # name -> [(t, value)]
pid_axes = Counter()

while True:
    msg = m.recv_match(blocking=False)
    if msg is None:
        break
    t = getattr(msg, "_timestamp", None)
    typ = msg.get_type()
    if typ == "BAD_DATA":
        continue
    counts[typ] += 1
    src[(msg.get_srcSystem(), msg.get_srcComponent(), typ)] += 1
    if first_t is None:
        first_t = t
    last_t = t
    if typ == "HEARTBEAT" and msg.get_srcComponent() == 1 and msg.type != 6:
        mode_str = mavutil.mode_string_v10(msg)
        armed = bool(msg.base_mode & 128)
        key = (mode_str, armed)
        if key != last_mode:
            modes.append((t, mode_str, armed))
            last_mode = key
    elif typ == "PARAM_VALUE":
        params[msg.param_id].append((t, msg.param_value))
    elif typ == "PID_TUNING":
        pid_axes[msg.axis] += 1

print(f"duration: {last_t - first_t:.1f} s  ({first_t:.0f} -> {last_t:.0f})")
print("\n== message counts (top 40) ==")
for k, v in counts.most_common(40):
    print(f"{k:28s} {v}")
print("\n== sources (sys, comp) ==")
by_src = Counter()
for (s, c, typ), v in src.items():
    by_src[(s, c)] += v
for k, v in by_src.most_common():
    print(k, v)
print("\n== PID_TUNING axes ==", dict(pid_axes))
print("\n== mode / arm transitions ==")
for t, ms, armed in modes:
    print(f"{t - first_t:8.1f}s  {ms:12s} armed={armed}")
print("\n== params seen in log with changes ==")
watch = [p for p in params if p.startswith(("ATC_STR", "ATC_SPEED", "ATC_ACCEL", "ATC_DECEL", "ATC_TURN",
                                            "WP_", "PSC_", "CRUISE", "MOT_", "ACRO_TURN", "TURN_RADIUS",
                                            "SERVO1_", "SERVO3_", "GCS_PID", "INS_GYRO_F", "AHRS_ORIENT",
                                            "FRAME_", "SCHED_LOOP", "LOIT_", "SPEED_MAX", "COMPASS_USE"))]
for p in sorted(watch):
    vals = params[p]
    uniq = []
    for t, v in vals:
        if not uniq or abs(uniq[-1][1] - v) > 1e-9:
            uniq.append((t, v))
    s = ", ".join(f"{v:g}@{t - first_t:.0f}s" for t, v in uniq)
    print(f"{p:20s} n={len(vals):3d}  {s}")
