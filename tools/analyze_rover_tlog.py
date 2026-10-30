#!/usr/bin/env python3
"""Analyse ArduPilot Rover steering behaviour from a MAVLink .tlog.

Read-only. Produces a text summary on stdout and PNG plots in an output directory.

usage: analyze_rover_tlog.py <file.tlog> [--out docs/img] [--no-plots]
"""
import argparse
import math
import os
from collections import defaultdict

import numpy as np
from pymavlink import mavutil

AUTOPILOT_COMP = 1
STEER_PWM_MIN, STEER_PWM_TRIM, STEER_PWM_MAX = 1200, 1500, 1800


def load(path):
    m = mavutil.mavlink_connection(path, dialect="ardupilotmega", robust_parsing=True)
    rows = defaultdict(list)
    modes = []
    last_mode = None
    params = {}
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            break
        typ = msg.get_type()
        if typ == "BAD_DATA" or msg.get_srcComponent() != AUTOPILOT_COMP:
            continue
        t = msg._timestamp
        if typ == "HEARTBEAT":
            if msg.type == 6:
                continue
            ms = mavutil.mode_string_v10(msg)
            if ms != last_mode:
                modes.append((t, ms))
                last_mode = ms
        elif typ == "PID_TUNING" and msg.axis == 5:
            rows["pid"].append((t, msg.desired, msg.achieved, msg.FF, msg.P, msg.I, msg.D, msg.SRate))
        elif typ == "ATTITUDE":
            rows["att"].append((t, msg.yaw, msg.yawspeed, msg.roll))
        elif typ == "VFR_HUD":
            rows["hud"].append((t, msg.groundspeed, msg.heading, msg.throttle))
        elif typ == "NAV_CONTROLLER_OUTPUT":
            # aspd_error carries Rover speed_error() = desired - actual (m/s)
            rows["nav"].append((t, msg.nav_bearing, msg.target_bearing, msg.wp_dist, msg.xtrack_error, msg.aspd_error))
        elif typ == "SERVO_OUTPUT_RAW":
            rows["servo"].append((t, msg.servo1_raw, msg.servo3_raw))
        elif typ == "RC_CHANNELS_SCALED":
            # Rover MSG_SERVO_OUT: chan1 = post-mixer steering (±10000), chan3 = throttle*100
            rows["mix"].append((t, msg.chan1_scaled / 10000.0, msg.chan3_scaled / 100.0))
        elif typ == "MISSION_CURRENT":
            rows["wp"].append((t, msg.seq))
        elif typ == "GLOBAL_POSITION_INT":
            rows["pos"].append((t, msg.lat / 1e7, msg.lon / 1e7))
        elif typ == "PARAM_VALUE":
            params.setdefault(msg.param_id, msg.param_value)
    arrays = {k: np.array(v, dtype=float) for k, v in rows.items()}
    return arrays, modes, params


def segments(modes, t_end):
    out = []
    for i, (t, ms) in enumerate(modes):
        t1 = modes[i + 1][0] if i + 1 < len(modes) else t_end
        out.append((t, t1, ms))
    return out


def sel(arr, t0, t1):
    return arr[(arr[:, 0] >= t0) & (arr[:, 0] < t1)]


def interp(src_t, src_v, dst_t):
    return np.interp(dst_t, src_t, src_v)


def zero_cross_period(t, x):
    """Mean period (s) from sign changes of a zero-mean signal."""
    x = x - np.mean(x)
    s = np.sign(x)
    idx = np.where(np.diff(s) != 0)[0]
    if len(idx) < 4:
        return float("nan")
    return 2.0 * np.mean(np.diff(t[idx]))


def psd_peak(t, x, fmax=1.0):
    """Dominant frequency (Hz) of a uniformly resampled signal."""
    if len(t) < 32:
        return float("nan"), float("nan")
    dt = np.median(np.diff(t))
    tu = np.arange(t[0], t[-1], dt)
    xu = np.interp(tu, t, x) - np.mean(x)
    xu *= np.hanning(len(xu))
    f = np.fft.rfftfreq(len(xu), dt)
    p = np.abs(np.fft.rfft(xu)) ** 2
    mask = (f > 0.02) & (f < fmax)
    if not mask.any():
        return float("nan"), float("nan")
    i = np.argmax(p[mask])
    return f[mask][i], p[mask][i] / p[mask].sum()


def band_rms(t, x, f_lo, f_hi):
    """RMS of x within [f_lo, f_hi] Hz."""
    if len(t) < 32:
        return float("nan")
    dt = np.median(np.diff(t))
    tu = np.arange(t[0], t[-1], dt)
    xu = np.interp(tu, t, x) - np.mean(x)
    f = np.fft.rfftfreq(len(xu), dt)
    X = np.fft.rfft(xu)
    X[(f < f_lo) | (f > f_hi)] = 0
    return float(np.sqrt(np.mean(np.fft.irfft(X, len(xu)) ** 2)))


def xcorr_lag(t, u, y, max_lag=8.0):
    """Lag (s) at which y best correlates with u (positive = y lags u)."""
    dt = np.median(np.diff(t))
    tu = np.arange(t[0], t[-1], dt)
    uu = np.interp(tu, t, u) - np.mean(u)
    yu = np.interp(tu, t, y) - np.mean(y)
    n = int(max_lag / dt)
    lags = np.arange(-n, n + 1)
    c = np.array([np.sum(uu[max(0, -k):len(uu) - max(0, k)] * yu[max(0, k):len(yu) - max(0, -k)]) for k in lags])
    norm = np.sqrt(np.sum(uu ** 2) * np.sum(yu ** 2))
    if norm == 0:
        return float("nan"), float("nan")
    c /= norm
    i = np.argmax(c)
    return lags[i] * dt, c[i]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tlog")
    ap.add_argument("--out", default="docs/img")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    A, modes, params = load(args.tlog)
    t0 = min(a[0, 0] for a in A.values())
    t_end = max(a[-1, 0] for a in A.values())
    segs = segments(modes, t_end)

    pid, att, hud, nav, servo, mix, wp = (A[k] for k in ("pid", "att", "hud", "nav", "servo", "mix", "wp"))
    pid_out = pid[:, 3] + pid[:, 4] + pid[:, 5] + pid[:, 6]

    print(f"log span {t_end - t0:.0f} s, PID_TUNING n={len(pid)} ({len(pid) / (t_end - t0):.1f} Hz), "
          f"ATTITUDE n={len(att)}, SERVO_OUTPUT_RAW n={len(servo)}, NAV n={len(nav)}")
    print(f"logged ATC_STR_RAT_P/I/FF/IMAX = {params.get('ATC_STR_RAT_P')}/{params.get('ATC_STR_RAT_I')}/"
          f"{params.get('ATC_STR_RAT_FF')}/{params.get('ATC_STR_RAT_IMAX')}, RAT_MAX={params.get('ATC_STR_RAT_MAX')}, "
          f"ACC_MAX={params.get('ATC_STR_ACC_MAX')}, FLTT/FLTE={params.get('ATC_STR_RAT_FLTT')}/{params.get('ATC_STR_RAT_FLTE')}, "
          f"SMAX={params.get('ATC_STR_RAT_SMAX')}")
    print(f"logged CRUISE_SPEED/THROTTLE={params.get('CRUISE_SPEED')}/{params.get('CRUISE_THROTTLE')}, "
          f"WP_SPEED={params.get('WP_SPEED')}, WP_ACCEL={params.get('WP_ACCEL')}, MOT_VEC_ANGLEMAX={params.get('MOT_VEC_ANGLEMAX')}, "
          f"MOT_SLEWRATE={params.get('MOT_SLEWRATE')}, TURN_RADIUS={params.get('TURN_RADIUS')}, WP_RADIUS={params.get('WP_RADIUS')}")

    # ---------------------------------------------------------------- per-segment stats
    print("\n== segments ==")
    hdr = ("seg", "mode", "start", "dur", "spd", "thr%", "tgt_rate", "act_rate", "rate_err", "out_rms", "P_rms", "I_rms",
           "FF_rms", "sat%", "xtrk_rms", "T_zc", "f_peak")
    print(" ".join(f"{h:>8s}" for h in hdr))
    auto_segs = []
    for i, (ta, tb, ms) in enumerate(segs):
        p = sel(pid, ta + 2, tb)
        h = sel(hud, ta + 2, tb)
        n = sel(nav, ta + 2, tb)
        s = sel(servo, ta + 2, tb)
        if len(p) < 20 or len(h) < 5:
            continue
        out = p[:, 3] + p[:, 4] + p[:, 5] + p[:, 6]
        sat = np.mean((s[:, 1] <= STEER_PWM_MIN + 5) | (s[:, 1] >= STEER_PWM_MAX - 5)) * 100 if len(s) else float("nan")
        xt = np.sqrt(np.mean(n[:, 4] ** 2)) if len(n) else float("nan")
        f_peak, _ = psd_peak(p[:, 0], out)
        row = (f"{i:8d}", f"{ms:>8s}", f"{ta - t0:8.0f}", f"{tb - ta:8.0f}", f"{np.mean(h[:, 1]):8.2f}",
               f"{np.mean(h[:, 3]):8.1f}", f"{np.sqrt(np.mean(p[:, 1] ** 2)):8.2f}", f"{np.sqrt(np.mean(p[:, 2] ** 2)):8.2f}",
               f"{np.sqrt(np.mean((p[:, 1] - p[:, 2]) ** 2)):8.2f}", f"{np.sqrt(np.mean(out ** 2)):8.3f}",
               f"{np.sqrt(np.mean(p[:, 4] ** 2)):8.3f}", f"{np.sqrt(np.mean(p[:, 5] ** 2)):8.3f}",
               f"{np.sqrt(np.mean(p[:, 3] ** 2)):8.3f}", f"{sat:8.1f}", f"{xt:8.2f}",
               f"{zero_cross_period(p[:, 0], out):8.1f}", f"{f_peak:8.3f}")
        print(" ".join(row))
        if ms == "AUTO":
            auto_segs.append((i, ta, tb))

    # ---------------------------------------------------------------- straight-line windows in AUTO
    # "straight" = |target turn rate| < 6 deg/s for the whole window and > 15 s since last WP change
    print("\n== AUTO straight-line windows (|target| < 6 deg/s, >15 s after WP change) ==")
    print(f"{'seg':>4s} {'t0':>7s} {'dur':>6s} {'spd':>5s} {'thr%':>5s} {'act_rms':>8s} {'hf_rms':>7s} {'out_rms':>8s} {'servo_pp':>8s} "
          f"{'xtrk_rms':>8s} {'T_zc':>6s} {'f_peak':>7s} {'lag_s':>6s} {'corr':>5s}")
    straight = []
    for (i, ta, tb) in auto_segs:
        wps = sel(wp, ta, tb)
        changes = [ta] + [wps[j, 0] for j in range(1, len(wps)) if wps[j, 1] != wps[j - 1, 1]]
        changes.append(tb)
        for j in range(len(changes) - 1):
            # S-curve blends into the next leg ~10 s before MISSION_CURRENT increments
            w0, w1 = changes[j] + 15, changes[j + 1] - 12
            if w1 - w0 < 20:
                continue
            p = sel(pid, w0, w1)
            if len(p) < 40 or np.max(np.abs(p[:, 1])) > 6.0:
                continue
            h = sel(hud, w0, w1)
            n = sel(nav, w0, w1)
            s = sel(servo, w0, w1)
            out = p[:, 3] + p[:, 4] + p[:, 5] + p[:, 6]
            lag, corr = xcorr_lag(p[:, 0], out, p[:, 2])
            f_peak, _ = psd_peak(p[:, 0], out)
            hf = band_rms(p[:, 0], p[:, 2], 0.25, 0.8)
            straight.append((i, w0, w1, hf))
            print(f"{i:4d} {w0 - t0:7.0f} {w1 - w0:6.0f} {np.mean(h[:, 1]):5.2f} {np.mean(h[:, 3]):5.1f} "
                  f"{np.sqrt(np.mean(p[:, 2] ** 2)):8.2f} {hf:7.2f} {np.sqrt(np.mean(out ** 2)):8.3f} "
                  f"{(np.ptp(s[:, 1]) if len(s) else float('nan')):8.0f} {np.sqrt(np.mean(n[:, 4] ** 2)) if len(n) else float('nan'):8.2f} "
                  f"{zero_cross_period(p[:, 0], out):6.1f} {f_peak:7.3f} {lag:6.2f} {corr:5.2f}")

    # ---------------------------------------------------------------- corners
    print("\n== AUTO waypoint transitions (25 s after each WP change) ==")
    print(f"{'seg':>4s} {'t':>7s} {'wp':>3s} {'hdg_chg':>7s} {'tgt_max':>7s} {'act_max':>7s} {'out_max':>7s} {'servo_min':>9s} "
          f"{'servo_max':>9s} {'sat_s':>5s} {'xtrk_max':>8s} {'xtrk_30s':>8s} {'t_settle':>8s}")
    for (i, ta, tb) in auto_segs:
        wps = sel(wp, ta, tb)
        for j in range(1, len(wps)):
            if wps[j, 1] == wps[j - 1, 1]:
                continue
            tc = wps[j, 0]
            p = sel(pid, tc, tc + 25)
            s = sel(servo, tc, tc + 25)
            n = sel(nav, tc, tc + 60)
            h0 = sel(hud, tc - 5, tc)
            h1 = sel(hud, tc + 20, tc + 30)
            if len(p) < 10 or len(n) < 3:
                continue
            out = p[:, 3] + p[:, 4] + p[:, 5] + p[:, 6]
            hdg_chg = float("nan")
            if len(h0) and len(h1):
                hdg_chg = (np.mean(h1[:, 2]) - np.mean(h0[:, 2]) + 180) % 360 - 180
            sat_s = np.sum((s[:, 1] <= STEER_PWM_MIN + 5) | (s[:, 1] >= STEER_PWM_MAX - 5)) * np.median(np.diff(s[:, 0])) if len(s) > 1 else float("nan")
            xt = n[:, 4]
            xt_max = np.max(np.abs(xt))
            n30 = sel(nav, tc + 25, tc + 35)
            xt_30 = np.sqrt(np.mean(n30[:, 4] ** 2)) if len(n30) else float("nan")
            settled = np.where(np.abs(xt) < 2.0)[0]
            # first time after the peak that |xtrack| stays < 2 m
            ipk = np.argmax(np.abs(xt))
            after = settled[settled > ipk]
            t_settle = n[after[0], 0] - tc if len(after) else float("nan")
            print(f"{i:4d} {tc - t0:7.0f} {int(wps[j, 1]):3d} {hdg_chg:7.0f} {np.max(np.abs(p[:, 1])):7.1f} "
                  f"{np.max(np.abs(p[:, 2])):7.1f} {np.max(np.abs(out)):7.3f} {np.min(s[:, 1]) if len(s) else 0:9.0f} "
                  f"{np.max(s[:, 1]) if len(s) else 0:9.0f} {sat_s:5.1f} {xt_max:8.1f} {xt_30:8.1f} {t_settle:8.1f}")

    # ---------------------------------------------------------------- vectored-thrust gain check
    print("\n== vectored-thrust mixer: post-mixer steering vs PID output (AUTO only) ==")
    for (i, ta, tb) in auto_segs:
        p = sel(pid, ta + 2, tb)
        mx = sel(mix, ta + 2, tb)
        if len(mx) < 10 or len(p) < 10:
            continue
        out_i = np.interp(mx[:, 0], p[:, 0], pid_out[(pid[:, 0] >= ta + 2) & (pid[:, 0] < tb)])
        thr = mx[:, 2] / 100.0
        ok = (np.abs(out_i) > 0.01) & (thr > 0.05)
        if ok.sum() < 10:
            continue
        gain = mx[ok, 1] / out_i[ok]
        print(f"seg {i}: median servo_norm/pid_out = {np.median(gain):.2f} (1/thr*1/0.611 predicts {np.median(1.0 / thr[ok] / math.radians(35)):.2f}), "
              f"thr median {np.median(thr[ok]) * 100:.0f}%")

    # ---------------------------------------------------------------- MANUAL plant response
    print("\n== MANUAL: servo PWM -> yaw rate (lag, gain) ==")
    for i, (ta, tb, ms) in enumerate(segs):
        if ms != "MANUAL" or tb - ta < 60:
            continue
        s = sel(servo, ta + 2, tb)
        a = sel(att, ta + 2, tb)
        h = sel(hud, ta + 2, tb)
        if len(s) < 50 or len(a) < 50:
            continue
        steer_norm = -(s[:, 1] - STEER_PWM_TRIM) / 300.0  # SERVO1_REVERSED=1
        yaw_rate = np.degrees(np.interp(s[:, 0], a[:, 0], a[:, 2]))
        moving = np.interp(s[:, 0], h[:, 0], h[:, 1]) > 1.0
        if moving.sum() < 50:
            continue
        lag, corr = xcorr_lag(s[moving, 0], steer_norm[moving], yaw_rate[moving])
        y = np.interp(s[moving, 0] + (lag if not math.isnan(lag) else 0), s[moving, 0], yaw_rate[moving])
        K = np.polyfit(steer_norm[moving], y, 1)[0]
        print(f"seg {i}: n={moving.sum()}, speed {np.mean(h[:, 1]):.2f} m/s, lag {lag:.2f} s (corr {corr:.2f}), "
              f"gain {K:.1f} deg/s per full servo deflection, |yaw_rate| 95th pct {np.percentile(np.abs(yaw_rate[moving]), 95):.1f} deg/s, "
              f"max {np.max(np.abs(yaw_rate[moving])):.1f}")

    # ---------------------------------------------------------------- is the 0.3-0.5 Hz wobble controller-induced?
    print("\n== yaw-rate 0.25-0.8 Hz band RMS by mode (ATTITUDE.yawspeed, speed > 1.5 m/s, 60 s windows) ==")
    for i, (ta, tb, ms) in enumerate(segs):
        for w0 in np.arange(ta + 5, tb - 60, 60):
            a = sel(att, w0, w0 + 60)
            h = sel(hud, w0, w0 + 60)
            if len(a) < 100 or len(h) < 10 or np.mean(h[:, 1]) < 1.5:
                continue
            yr = np.degrees(a[:, 2])
            fpk, frac = psd_peak(a[:, 0], yr, fmax=1.5)
            print(f"seg {i:2d} {ms:7s} t={w0 - t0:6.0f}s spd={np.mean(h[:, 1]):4.2f} thr={np.mean(h[:, 3]):4.1f}% "
                  f"yaw_rate rms={np.sqrt(np.mean(yr ** 2)):5.2f} band(0.25-0.8Hz) rms={band_rms(a[:, 0], yr, 0.25, 0.8):5.2f} "
                  f"peak {fpk:4.2f} Hz ({frac * 100:3.0f}% of power)")

    # ---------------------------------------------------------------- speed loop
    print("\n== speed loop (AUTO) ==")
    for (i, ta, tb) in auto_segs:
        h = sel(hud, ta + 5, tb)
        n = sel(nav, ta + 5, tb)
        if len(h) < 10:
            continue
        fpk, _ = psd_peak(h[:, 0], h[:, 3], fmax=0.2)
        print(f"seg {i}: groundspeed mean {np.mean(h[:, 1]):.2f} (min {np.min(h[:, 1]):.2f} max {np.max(h[:, 1]):.2f}), "
              f"throttle mean {np.mean(h[:, 3]):.1f}% range {np.min(h[:, 3]):.0f}-{np.max(h[:, 3]):.0f}%, "
              f"throttle dominant period {1 / fpk if fpk else float('nan'):.0f} s, "
              f"NAV aspd_error all-zero: {bool(len(n)) and not np.any(n[:, 5])}")

    # ---------------------------------------------------------------- global yaw-rate capability
    a_all = att
    h_sp = np.interp(a_all[:, 0], hud[:, 0], hud[:, 1])
    mv = h_sp > 1.5
    print(f"\nyaw-rate capability (speed>1.5 m/s): 99th pct {np.percentile(np.abs(np.degrees(a_all[mv, 2])), 99):.1f} deg/s, "
          f"max {np.max(np.abs(np.degrees(a_all[mv, 2]))):.1f} deg/s")

    # ---------------------------------------------------------------- servo slew
    ds = np.abs(np.diff(servo[:, 1])) / np.diff(servo[:, 0])
    print(f"commanded steering PWM slew: 95th pct {np.percentile(ds, 95):.0f} us/s, max {np.max(ds):.0f} us/s "
          f"(full range 600 us); SERVO_OUTPUT_RAW rate {1 / np.median(np.diff(servo[:, 0])):.1f} Hz")

    if args.no_plots:
        return

    # ---------------------------------------------------------------- plots
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(args.out, exist_ok=True)
    for (i, ta, tb) in auto_segs:
        p = sel(pid, ta, tb)
        h = sel(hud, ta, tb)
        n = sel(nav, ta, tb)
        s = sel(servo, ta, tb)
        wps = sel(wp, ta, tb)
        out = p[:, 3] + p[:, 4] + p[:, 5] + p[:, 6]
        fig, ax = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
        tt = p[:, 0] - t0
        ax[0].plot(tt, p[:, 1], label="target rate (deg/s)")
        ax[0].plot(tt, p[:, 2], label="achieved rate (deg/s)", alpha=0.8)
        ax[0].set_ylabel("deg/s")
        ax[0].legend(loc="upper right")
        ax[1].plot(tt, p[:, 3], label="FF")
        ax[1].plot(tt, p[:, 4], label="P")
        ax[1].plot(tt, p[:, 5], label="I")
        ax[1].plot(tt, out, "k", label="sum", alpha=0.6)
        ax[1].set_ylabel("steer out (norm)")
        ax[1].legend(loc="upper right")
        ax[2].plot(s[:, 0] - t0, s[:, 1], label="steer servo PWM")
        ax[2].axhline(STEER_PWM_MIN, color="r", ls="--", lw=0.8)
        ax[2].axhline(STEER_PWM_MAX, color="r", ls="--", lw=0.8)
        ax[2].set_ylabel("us")
        ax[2].legend(loc="upper right")
        ax[3].plot(n[:, 0] - t0, n[:, 4], label="xtrack error (m)")
        ax[3].axhline(0, color="k", lw=0.5)
        ax[3].set_ylabel("m")
        ax[3].legend(loc="upper right")
        ax[4].plot(h[:, 0] - t0, h[:, 1], label="groundspeed (m/s)")
        ax[4].plot(h[:, 0] - t0, h[:, 3] / 10.0, label="throttle (%/10)")
        ax[4].set_ylabel("m/s | %/10")
        ax[4].set_xlabel("time since log start (s)")
        ax[4].legend(loc="upper right")
        for j in range(1, len(wps)):
            if wps[j, 1] != wps[j - 1, 1]:
                for a_ in ax:
                    a_.axvline(wps[j, 0] - t0, color="g", ls=":", lw=0.8)
        fig.suptitle(f"AUTO segment {i}: {ta - t0:.0f}-{tb - t0:.0f} s (green = waypoint change)")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, f"auto_seg{i}.png"), dpi=110)
        plt.close(fig)

    # straight-line zooms: longest window (slow weave) and most oscillatory window (limit cycle)
    picks = []
    if straight:
        picks.append(("straight_weave.png", max(straight, key=lambda x: x[2] - x[1])))
        picks.append(("straight_limitcycle.png", max(straight, key=lambda x: x[3])))
    for fname, (i, w0, w1, _) in picks:
        p = sel(pid, w0, w1)
        s = sel(servo, w0, w1)
        n = sel(nav, w0, w1)
        out = p[:, 3] + p[:, 4] + p[:, 5] + p[:, 6]
        fig, ax = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
        ax[0].plot(p[:, 0] - t0, p[:, 1], label="target rate")
        ax[0].plot(p[:, 0] - t0, p[:, 2], label="achieved rate")
        ax[0].set_ylabel("deg/s")
        ax[0].legend()
        ax[1].plot(p[:, 0] - t0, out, "k", label="PID out")
        ax[1].plot(p[:, 0] - t0, p[:, 4], label="P")
        ax[1].plot(p[:, 0] - t0, p[:, 3], label="FF")
        ax[1].plot(p[:, 0] - t0, p[:, 5], label="I")
        ax[1].set_ylabel("norm")
        ax[1].legend(loc="lower right")
        ax2 = ax[1].twinx()
        ax2.plot(s[:, 0] - t0, s[:, 1], "r", alpha=0.5, label="servo PWM")
        ax2.set_ylabel("us")
        ax[2].plot(n[:, 0] - t0, n[:, 4], label="xtrack (m)")
        ax[2].legend()
        ax[2].set_xlabel("time since log start (s)")
        fig.suptitle(f"Straight-line window, AUTO seg {i}: {w0 - t0:.0f}-{w1 - t0:.0f} s")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, fname), dpi=110)
        plt.close(fig)

    # ground track
    pos = A["pos"]
    fig, ax = plt.subplots(figsize=(9, 9))
    lat0 = np.mean(pos[:, 1])
    x = (pos[:, 2] - np.mean(pos[:, 2])) * 111320 * math.cos(math.radians(lat0))
    y = (pos[:, 1] - lat0) * 110540
    mode_at = np.zeros(len(pos), dtype=int)
    for k, (ta, tb, ms) in enumerate(segs):
        mode_at[(pos[:, 0] >= ta) & (pos[:, 0] < tb)] = 1 if ms == "AUTO" else 0
    ax.plot(x[mode_at == 0], y[mode_at == 0], ".", ms=2, color="0.7", label="MANUAL")
    ax.plot(x[mode_at == 1], y[mode_at == 1], ".", ms=2, color="C0", label="AUTO")
    ax.set_aspect("equal")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.legend()
    ax.set_title("ground track")
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, "ground_track.png"), dpi=110)
    plt.close(fig)
    print(f"\nplots written to {args.out}/")


if __name__ == "__main__":
    main()
