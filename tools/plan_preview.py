"""Plan the field for one tractor, check it, and draw it.  python3 tools/plan_preview.py [tractor_id]"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "jbj_sim"))
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from jbj_sim import planner, spec  # noqa: E402

tid, t, g = spec.tractor(sys.argv[1] if len(sys.argv) > 1 else None, ROOT / "specs")
kit, field = spec.load("kit", ROOT / "specs"), spec.load("field", ROOT / "specs")
plan, chk = planner.plan_field_fit(field, kit, g)

p = plan["path"]
print(f"{tid}: headland passes={plan["headland_passes"]} swaths={plan["n_swaths"]} angle={plan['angle_deg']:.1f} deg, path {chk['length_m']:.0f} m, "
      f"min turn radius {chk['min_radius_m']:.2f} m (tractor min {g['r_min_rear_axle']:.2f} m), "
      f"footprint max outside field edge {chk['max_outside_m']:.2f} m (allowed {kit["safety"]["geofence_margin_m"]} m) worst at {chk["worst_where"]}")
print("turn types:", sorted(set(k.split(':')[1] for k in plan['kinds'] if ':' in k and not k.startswith(('swath', 'headland')))))
work_len = sum(math.hypot(p[i + 1, 0] - p[i, 0], p[i + 1, 1] - p[i, 1]) for i in range(len(p) - 1) if p[i, 4] > 0.5)
t_est = sum(math.hypot(p[i + 1, 0] - p[i, 0], p[i + 1, 1] - p[i, 1]) / max(p[i, 5], 0.2) for i in range(len(p) - 1))
print(f"planting distance {work_len:.0f} m, estimated time {t_est / 60:.1f} min, field {plan['field'].area / 1e4:.2f} ha")
fig, ax = plt.subplots(figsize=(11, 7.5))
fx, fy = plan["field"].exterior.xy
ax.fill(fx, fy, color="#d9c3a5", zorder=0)
ix, iy = plan["inner_area"].exterior.xy
ax.plot(ix, iy, "--", color="#8a6d47", lw=1, label="headland edge")
w = p[:, 4] > 0.5
ax.plot(p[:, 0], p[:, 1], color="#888", lw=0.8, label="turns / transitions")
pw = p.copy()
pw[~w, :2] = math.nan
ax.plot(pw[:, 0], pw[:, 1], color="#1a7f37", lw=1.4, label="planting (planter down)")
ax.plot(p[0, 0], p[0, 1], "o", color="k", label="start")
ax.set_aspect("equal")
ax.set_title(f"Coverage plan: {t['name']} + {kit['implement']['name']} ({kit['implement']['working_width_m']} m), "
             f"{plan['field'].area / 1e4:.2f} ha")
ax.set_xlabel("east (m)")
ax.set_ylabel("north (m)")
ax.legend(loc="upper right", fontsize=8)
out = ROOT / "media" / "plan_preview.png"
fig.savefig(out, dpi=110, bbox_inches="tight")
print("saved", out)
