"""Fast offline check of the steering loop (no Gazebo): bicycle model + the same actuator model, GNSS rate/latency
and pure pursuit as the sim nodes. Used to pick the look-ahead and latency compensation.

  python3 tools/tune_follower.py
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "jbj_sim"))
from jbj_sim import planner, spec  # noqa: E402
from jbj_sim.common import PathTracker, wrap  # noqa: E402

tid, t, g = spec.tractor(None, ROOT / "specs")
kit, field = spec.load("kit", ROOT / "specs"), spec.load("field", ROOT / "specs")
P = planner.plan_field(field, kit, g)["path"]
P = P[: int(np.searchsorted(np.cumsum(np.r_[0, np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))]), 900.0))]  # first ~900 m: swaths + turns
L, dmax = g["L"], 0.95 * g["delta_eq_max"]
rate, tau = math.radians(kit["steering"]["front_wheel_rate_deg_s"]), kit["steering"]["steer_response_s"]


def run(gain, ld_min, t_pred, plant_lag=0.10, seed=1):
    rng = np.random.default_rng(seed)
    x, y, yaw = P[0, 0], P[0, 1], P[0, 2]
    v, d_rl, d, d_plant = 0.0, 0.0, 0.0, 0.0
    trk, trk_true = PathTracker(P), PathTracker(P)
    err, cmd = np.zeros(2), (0.0, 0.0)
    hist, xs = [], []
    dt = 0.01
    for k in range(int(900 / 1.5 / dt)):
        tk = k * dt
        if k % 10 == 0 and len(hist) >= 10:      # 10 Hz GNSS fix, 0.1 s old when it arrives
            hx, hy, hyaw = hist[-10]
            err = math.exp(-0.1 / 60) * err + rng.normal(0, 0.01 * math.sqrt(1 - math.exp(-0.2 / 60)), 2)
            mx, my = hx + err[0], hy + err[1]
            myaw = hyaw + rng.normal(0, math.radians(0.15))
            # latency compensation: project the fix forward by t_pred using the current speed/steer
            mx += v * math.cos(myaw) * t_pred
            my += v * math.sin(myaw) * t_pred
            myaw += v * math.tan(d) / L * t_pred
            i = trk.update(mx, my)
            vr = float(P[i, 5])
            ld = max(ld_min, gain * max(vr, 0.6))
            j = trk.point_ahead(ld)
            a = wrap(math.atan2(P[j, 1] - my, P[j, 0] - mx) - myaw)
            cmd = (float(np.clip(math.atan2(2 * L * math.sin(a), max(math.hypot(P[j, 0] - mx, P[j, 1] - my), 0.5)), -dmax, dmax)), vr)
            if i >= len(P) - 3:
                break
        dc, vc = cmd
        d_rl += max(-rate * dt, min(rate * dt, dc - d_rl))
        d += (d_rl - d) * min(1, dt / tau)
        d_plant += (d - d_plant) * min(1, dt / plant_lag)      # Gazebo steering-joint tracking (assumed)
        dv = vc - v
        v += max(-1.5 * dt, min(0.5 * dt, dv))
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v * math.tan(d_plant) / L * dt
        hist.append((x, y, yaw))
        if k % 10 == 0:
            ti = trk_true.update(x, y)
            if P[ti, 4] > 0.5 and v > 0.3:
                xs.append(abs(trk_true.xte(x, y)))
    e = np.array(xs) * 100
    return e.mean(), np.percentile(e, 95), e.max()


print(f"{tid}: L={L:.2f} m, steering {math.degrees(rate):.0f} deg/s, lag {tau}s")
if "lag" in sys.argv:   # how much does a slow steering joint in the simulator hurt?
    for lag in (0.1, 0.3, 0.6, 1.0):
        for gain, ld_min, tp in ((1.4, 2.5, 0.0), (1.4, 2.5, 0.25)):
            m, p95, mx = run(gain, ld_min, tp, plant_lag=lag)
            print(f"steer-joint lag {lag:.1f}s | lookahead {gain}s/{ld_min}m predict {tp}s -> mean {m:5.2f} p95 {p95:5.2f} max {mx:6.2f} cm")
    sys.exit()
for gain, ld_min, tp in [(1.4, 2.5, 0.0), (2.0, 3.5, 0.0), (2.5, 4.5, 0.0), (1.4, 2.5, 0.25), (2.0, 3.5, 0.25), (2.5, 4.0, 0.25), (3.0, 5.0, 0.25)]:
    m, p95, mx = run(gain, ld_min, tp)
    print(f"lookahead {gain:.1f}s (min {ld_min} m), predict {tp:.2f}s  ->  |xte| planting mean {m:5.2f} cm, p95 {p95:5.2f} cm, max {mx:5.2f} cm")
