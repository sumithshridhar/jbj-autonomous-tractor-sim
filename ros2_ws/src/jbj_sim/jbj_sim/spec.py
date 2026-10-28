"""Shared spec loader + tractor geometry. Pure Python: used by Blender, the model generator and the ROS nodes.

Frame: origin on the ground under the rear-axle centre, X forward, Y left, Z up (metres, radians).
"""
import json
import math
import os
from pathlib import Path


def specs_dir():
    env = os.environ.get("JBJ_ROOT")
    if env:
        return Path(env) / "specs"
    return Path(__file__).resolve().parents[4] / "specs"   # repo/ros2_ws/src/jbj_sim/jbj_sim/spec.py


def load(name, root=None):
    return json.loads((Path(root) if root else specs_dir()).joinpath(f"{name}.json").read_text())


def tyre_dims(size):
    """ISO metric tyre code like '460/85 R38' -> outer diameter d, section width w, rim diameter (m)."""
    w, rest = size.replace(" ", "").split("/")
    aspect, rim = rest.split("R")
    w = int(w) / 1000.0
    rim_d = float(rim) * 0.0254
    return {"d": rim_d + 2 * w * int(aspect) / 100.0, "w": w, "rim_d": rim_d}


def tractor_geometry(t, track):
    """Key dimensions from a tractors.json entry and the wheel track setting."""
    fr, rr = tyre_dims(t["tyre_front"]), tyre_dims(t["tyre_rear"])
    L = t["wheelbase_m"]
    x_front = max(t["overall_length_m"] - 0.95, L + 0.75)    # rear linkage arms reach ~0.95 m behind the rear axle
    delta_i = math.radians(t["steering_angle_deg"])          # inner front wheel at full lock (published: 55 deg)
    r_rear_centre = L / math.tan(delta_i) + track / 2.0       # Ackermann: radius of the rear-axle centre at full lock
    delta_eq = math.atan(L / r_rear_centre)                   # equivalent bicycle-model steering angle at full lock
    r_outer = math.hypot(r_rear_centre + track / 2.0 + fr["w"] / 2.0, L)
    return {"L": L, "track": track, "front": fr, "rear": rr, "x_front": x_front,
            "r_min_rear_axle": r_rear_centre, "delta_eq_max": delta_eq, "r_outer_front": r_outer,
            "h_roof": t["height_roof_m"], "clearance": t["ground_clearance_m"],
            "hitch_x": -0.95, "lidar_xyz": (x_front - 0.02, 0.0, 1.30)}


def track_of(t, tractors):
    """Wheel track used for a tractor: its own track_m setting, else the row-crop default."""
    return t.get("track_m", tractors["row_crop_track_m"])


def tractor(tid=None, root=None):
    tr = load("tractors", root)
    tid = tid or tr["default"]
    t = tr["models"][tid]
    return tid, t, tractor_geometry(t, track_of(t, tr))


def implement_rear_x(kit):
    """x of the planter's rear (closing wheels) in the tractor frame; the working line is the opener discs."""
    return -0.95 - kit["implement"]["length_behind_hitch_m"]


def implement_work_x():
    """x where seed goes into the ground (opener discs), tractor frame."""
    return -0.95 - 0.85
