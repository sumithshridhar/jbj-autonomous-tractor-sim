"""Shared runtime helpers: WGS84 <-> local ENU, plan file I/O, path tracking."""
import json
import math
from pathlib import Path

import numpy as np

# WGS84
_A = 6378137.0
_F = 1 / 298.257223563
_E2 = _F * (2 - _F)


def _ecef(lat, lon, h):
    la, lo = math.radians(lat), math.radians(lon)
    n = _A / math.sqrt(1 - _E2 * math.sin(la) ** 2)
    return ((n + h) * math.cos(la) * math.cos(lo), (n + h) * math.cos(la) * math.sin(lo), (n * (1 - _E2) + h) * math.sin(la))


def lla_to_enu(lat, lon, h, lat0, lon0, h0):
    x, y, z = _ecef(lat, lon, h)
    x0, y0, z0 = _ecef(lat0, lon0, h0)
    dx, dy, dz = x - x0, y - y0, z - z0
    la, lo = math.radians(lat0), math.radians(lon0)
    e = -math.sin(lo) * dx + math.cos(lo) * dy
    n = -math.sin(la) * math.cos(lo) * dx - math.sin(la) * math.sin(lo) * dy + math.cos(la) * dz
    u = math.cos(la) * math.cos(lo) * dx + math.cos(la) * math.sin(lo) * dy + math.sin(la) * dz
    return e, n, u


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def save_plan(run_dir, plan, meta):
    run_dir = Path(run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    np.savez(run_dir / "plan.npz", path=plan["path"], field=np.array(plan["field"].exterior.coords),
             inner=np.array(plan["inner_area"].exterior.coords))
    meta = dict(meta, kinds=plan["kinds"], angle_deg=plan["angle_deg"], n_swaths=plan["n_swaths"],
                width=plan["width"], radius=plan["radius"], headland_m=plan["headland_m"], work_back=plan["work_back"])
    (run_dir / "plan.json").write_text(json.dumps(meta, indent=1))


def load_plan(run_dir):
    d = np.load(Path(run_dir) / "plan.npz")
    meta = json.loads((Path(run_dir) / "plan.json").read_text())
    return d["path"], d["field"], d["inner"], meta


class PathTracker:
    """Finds the nearest path point (searching forward from the last one) and the signed cross-track error."""

    def __init__(self, path, window_back=40, window_fwd=600):
        self.p = path
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1])))])
        self.idx = 0
        self.wb, self.wf = window_back, window_fwd

    def update(self, x, y):
        lo, hi = max(0, self.idx - self.wb), min(len(self.p), self.idx + self.wf)
        seg = self.p[lo:hi]
        d2 = (seg[:, 0] - x) ** 2 + (seg[:, 1] - y) ** 2
        self.idx = lo + int(np.argmin(d2))
        return self.idx

    def xte(self, x, y):
        """Signed lateral error (left of path = positive) to the segment around the nearest point."""
        i = min(self.idx, len(self.p) - 2)
        best = None
        for j in (max(0, i - 1), i):
            x0, y0, x1, y1 = self.p[j, 0], self.p[j, 1], self.p[j + 1, 0], self.p[j + 1, 1]
            dx, dy = x1 - x0, y1 - y0
            L2 = dx * dx + dy * dy
            if L2 < 1e-12:
                continue
            t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
            px, py = x0 + t * dx, y0 + t * dy
            d = math.hypot(x - px, y - py)
            sign = 1.0 if (dx * (y - y0) - dy * (x - x0)) >= 0 else -1.0
            if best is None or d < abs(best):
                best = sign * d
        return 0.0 if best is None else best

    def point_ahead(self, dist):
        target = self.s[self.idx] + dist
        j = int(np.searchsorted(self.s, target))
        return min(j, len(self.p) - 1)


def footprint_corners(x, y, yaw, x_front, x_rear, half_w):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in
            [(x_front, half_w), (x_front, -half_w), (x_rear, -half_w), (x_rear, half_w)]]
