"""Coverage path planner for a tractor + mounted planter.

Method (the usual one for row crops, as in Fields2Cover):
  1. Headland = field shrunk by (headland passes x working width). The inner area is planted first with straight
     back-and-forth passes parallel to the longest field edge, visited in skip-one order (1, 3, 5, ... then back
     ..., 4, 2) so most turns are simple U-turns.
  2. Every turn is a Dubins path (shortest forward-only path with a minimum turning radius).
  3. The headland is planted last: one lap per headland pass, corners rounded to the turn radius.
The path is for the rear-axle centre. Each point carries a 'work' flag (planter down) worked out for the
opener discs, which sit behind the rear axle.
"""
import math

import numpy as np
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon

TWO_PI = 2 * math.pi


def mod2pi(a):
    return a - TWO_PI * math.floor(a / TWO_PI)


# ------------------------------------------------------------------ Dubins
def _dubins_words(d, a, b):
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    cab = math.cos(a - b)
    out = {}
    p2 = 2 + d * d - 2 * cab + 2 * d * (sa - sb)
    if p2 >= 0:
        t1 = math.atan2(cb - ca, d + sa - sb)
        out["LSL"] = (mod2pi(-a + t1), math.sqrt(p2), mod2pi(b - t1))
    p2 = 2 + d * d - 2 * cab + 2 * d * (sb - sa)
    if p2 >= 0:
        t1 = math.atan2(ca - cb, d - sa + sb)
        out["RSR"] = (mod2pi(a - t1), math.sqrt(p2), mod2pi(-b + t1))
    p2 = -2 + d * d + 2 * cab + 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        t2 = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out["LSR"] = (mod2pi(-a + t2), p, mod2pi(-mod2pi(b) + t2))
    p2 = -2 + d * d + 2 * cab - 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        t2 = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out["RSL"] = (mod2pi(a - t2), p, mod2pi(b - t2))
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1:
        p = mod2pi(TWO_PI - math.acos(tmp))
        t = mod2pi(a - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        out["RLR"] = (t, p, mod2pi(a - b - t + p))
    tmp = (6.0 - d * d + 2 * cab + 2 * d * (-sa + sb)) / 8.0
    if abs(tmp) <= 1:
        p = mod2pi(TWO_PI - math.acos(tmp))
        t = mod2pi(-a - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        out["LRL"] = (t, p, mod2pi(mod2pi(b) - a - t + p))
    return out


def sample_segments(q0, segs, ds):
    """segs: list of (curvature, length). Exact arc integration. Returns array [x, y, yaw, curvature]."""
    x, y, th = q0
    pts = [(x, y, th, segs[0][0] if segs else 0.0)]
    for k, length in segs:
        if length <= 1e-9:
            continue
        n = max(1, int(math.ceil(length / ds)))
        step = length / n
        for _ in range(n):
            if abs(k) < 1e-12:
                x += step * math.cos(th)
                y += step * math.sin(th)
            else:
                dth = step * k
                x += (math.sin(th + dth) - math.sin(th)) / k
                y += (-math.cos(th + dth) + math.cos(th)) / k
                th += dth
            pts.append((x, y, th, k))
    return np.array(pts)


def dubins(q0, q1, R, ds=0.25):
    """Shortest Dubins path q0 -> q1 (x, y, yaw), radius R. Every candidate is checked by sampling its end pose."""
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    d = math.hypot(dx, dy) / R
    th = mod2pi(math.atan2(dy, dx)) if d > 1e-9 else 0.0
    a, b = mod2pi(q0[2] - th), mod2pi(q1[2] - th)
    best = None
    for word, (t, p, q) in _dubins_words(d, a, b).items():
        kk = {"L": 1.0 / R, "R": -1.0 / R, "S": 0.0}
        segs = [(kk[word[0]], t * R), (kk[word[1]], p * R), (kk[word[2]], q * R)]
        total = (t + p + q) * R
        if best is not None and total >= best[0]:
            continue
        pts = sample_segments(q0, segs, ds)
        end = pts[-1]
        err = math.hypot(end[0] - q1[0], end[1] - q1[1]) + abs(math.atan2(math.sin(end[2] - q1[2]), math.cos(end[2] - q1[2])))
        if err < 1e-3:
            best = (total, pts, word)
    if best is None:
        raise ValueError(f"no Dubins path from {q0} to {q1}")
    return best[1], best[2]


# ------------------------------------------------------------------ helpers
def _straight(p0, p1, ds):
    yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    return sample_segments((p0[0], p0[1], yaw), [(0.0, math.hypot(p1[0] - p0[0], p1[1] - p0[1]))], ds)


def _ring_path(poly, R, ds, ccw=True):
    """Closed lap around a convex polygon with corners rounded to radius R. Returns points starting mid first edge."""
    xy = list(poly.exterior.coords)[:-1]
    if Polygon(xy).exterior.is_ccw != ccw:
        xy = xy[::-1]
    n = len(xy)
    P = [np.array(p) for p in xy]
    segs, start = [], None
    for i in range(n):
        p_prev, p, p_next = P[i - 1], P[i], P[(i + 1) % n]
        u_in = (p - p_prev) / np.linalg.norm(p - p_prev)
        u_out = (p_next - p) / np.linalg.norm(p_next - p)
        phi = math.atan2(u_in[0] * u_out[1] - u_in[1] * u_out[0], float(np.dot(u_in, u_out)))  # signed turn angle
        T = R * math.tan(abs(phi) / 2)
        segs.append((p - T * u_in, p + T * u_out, phi))
    pts = []
    for i in range(n):
        a_end = segs[i][1]                      # end of corner i arc = start of the straight to corner i+1
        b_start = segs[(i + 1) % n][0]
        yaw = math.atan2(b_start[1] - a_end[1], b_start[0] - a_end[0])
        straight = sample_segments((a_end[0], a_end[1], yaw), [(0.0, float(np.linalg.norm(b_start - a_end)))], ds)
        phi = segs[(i + 1) % n][2]
        arc = sample_segments(tuple(straight[-1][:3]), [(math.copysign(1.0 / R, phi), R * abs(phi))], ds)
        pts.append(straight if not pts else straight[1:])
        pts.append(arc[1:])
    ring = np.vstack(pts)
    # start the lap halfway along the first straight
    mid = len(pts[0]) // 2
    return np.vstack([ring[mid:], ring[1:mid + 1]])


def _arc_lengths(xy):
    d = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    return np.concatenate([[0.0], np.cumsum(d)])


def speed_profile(path, v_work, v_turn, a_up, a_down, kappa_turn=0.02):
    s = _arc_lengths(path[:, :2])
    v = np.where(path[:, 4] > 0.5, v_work, v_turn)
    v = np.where(np.abs(path[:, 3]) > kappa_turn, np.minimum(v, v_turn), v)
    v[-1] = 0.0
    for i in range(1, len(v)):          # acceleration limit
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * a_up * (s[i] - s[i - 1])))
    for i in range(len(v) - 2, -1, -1):  # braking limit
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * a_down * (s[i + 1] - s[i])))
    v[0] = max(v[0], 0.3)
    return v


def footprint(x, y, yaw, geom, kit):
    """Tractor body rectangle + planter rectangle, as one polygon in the field frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    tr_half = (geom["track"] + geom["rear"]["w"]) / 2 + 0.05
    im_half = kit["implement"]["working_width_m"] / 2 + 0.05
    x_rear = -0.95 - kit["implement"]["length_behind_hitch_m"]

    def rect(x0, x1, hw):
        return Polygon([(x + c * px - s * py, y + s * px + c * py)
                        for px, py in [(x1, hw), (x1, -hw), (x0, -hw), (x0, hw)]])
    return rect(-0.95, geom["x_front"], tr_half).union(rect(x_rear, -0.95, im_half))   # they touch at the hitch


# ------------------------------------------------------------------ main planner
def plan_field(field, kit, geom, ds=0.25):
    """Returns dict with 'path' array [x, y, yaw, curvature, work, v_ref, seg_id], segments info and checks."""
    im = kit["implement"]
    w = im["working_width_m"]
    R = field["turn_radius_m"]
    passes = field["headland_passes"]
    H = passes * w
    work_back = 0.95 + 0.85            # rear axle -> opener discs (m), see spec.implement_work_x()
    lead_in = 4.0                      # straight before the planter goes down, to settle on the line
    F = Polygon(field["boundary_enu_m"])
    A = F.buffer(-H, join_style=2)
    if field.get("swath_angle_deg") is None:
        xy = list(F.exterior.coords)
        edges = [(math.hypot(x1 - x0, y1 - y0), math.atan2(y1 - y0, x1 - x0)) for (x0, y0), (x1, y1) in zip(xy, xy[1:])]
        ang = max(edges)[1]
    else:
        ang = math.radians(field["swath_angle_deg"])
    deg = math.degrees(ang)
    Ar = affinity.rotate(A, -deg, origin=(0, 0))
    minx, miny, maxx, maxy = Ar.bounds
    n = int(math.ceil((maxy - miny) / w - 1e-6))
    ys = [miny + w / 2 + k * w for k in range(n)]
    if n > 1:
        ys[-1] = min(ys[-1], maxy - w / 2)     # last pass overlaps the one before instead of sticking out
    swaths = []
    for yv in ys:
        seg = LineString([(minx - 1, yv), (maxx + 1, yv)]).intersection(Ar)
        (xa, _), (xb, _) = list(seg.coords)[0], list(seg.coords)[-1]
        swaths.append((min(xa, xb), max(xa, xb), yv))
    order = list(range(0, n, 2)) + list(reversed(range(1, n, 2)))

    c, s_ = math.cos(ang), math.sin(ang)

    def to_field(arr):
        out = arr.copy()
        out[:, 0] = c * arr[:, 0] - s_ * arr[:, 1]
        out[:, 1] = s_ * arr[:, 0] + c * arr[:, 1]
        out[:, 2] = arr[:, 2] + ang
        return out

    allowed = F.buffer(kit["safety"]["geofence_margin_m"] - 0.4)   # keep 0.4 m for path-tracking error
    r_floor = 1.15 * geom["r_min_rear_axle"]     # never plan tighter than 1.15 x the tractor's full-lock radius
    radii = [r for r in (R, 3.5, 3.0) if r >= r_floor - 1e-9] or [max(R, r_floor)]

    def turn(q0, q1):
        """Dubins turn with the largest radius whose swept tractor+planter footprint stays inside the geofence."""
        last = None
        for r in radii:
            pts, word = dubins(q0, q1, r, ds)
            last = (pts, f"{word}@{r:g}m")
            if all(allowed.contains(footprint(p[0], p[1], p[2], geom, kit)) for p in pts[::4]):
                return last
        return last

    pieces = []   # (array [x, y, yaw, k], work_mask or None, kind)
    prev_end = None
    for visit, i in enumerate(order):
        xa, xb, yv = swaths[i]
        if visit % 2 == 0:   # +x
            r0, r1, wa, wb = xa + work_back - lead_in, xb + work_back, xa + work_back, xb + work_back
            p0, p1 = (r0, yv), (r1, yv)
        else:                # -x
            r0, r1, wa, wb = xb - work_back + lead_in, xa - work_back, xa - work_back, xb - work_back
            p0, p1 = (r0, yv), (r1, yv)
        line = to_field(_straight(p0, p1, ds))
        xr = _straight(p0, p1, ds)[:, 0]
        work = (xr >= min(wa, wb) - 1e-6) & (xr <= max(wa, wb) + 1e-6)
        if prev_end is not None:
            tp, word = turn(tuple(prev_end[:3]), tuple(line[0][:3]))
            pieces.append((tp[1:], None, f"turn:{word}"))
        pieces.append((line, work, f"swath:{i}"))
        prev_end = line[-1]

    # headland laps, innermost first, direction chosen to give the shorter first transition
    laps_ccw = []
    for ccw in (True, False):
        rings = [_ring_path(F.buffer(-(H - (k + 0.5) * w), join_style=2), R, ds, ccw) for k in range(passes)]
        first = rings[0]
        idx = int(np.argmin(np.hypot(first[:, 0] - prev_end[0], first[:, 1] - prev_end[1])))
        idx = (idx + int(20.0 / ds)) % len(first)
        rings[0] = np.vstack([first[idx:], first[1:idx + 1]])
        t0, _ = dubins(tuple(prev_end[:3]), tuple(rings[0][0][:3]), R, ds)
        laps_ccw.append((_arc_lengths(t0[:, :2])[-1], rings))
    rings = min(laps_ccw, key=lambda z: z[0])[1]
    for k, ring in enumerate(rings):
        if k > 0:   # lane change to the next lap outwards, starting 25 m further along
            nxt = ring
            idx = int(np.argmin(np.hypot(nxt[:, 0] - prev_end[0], nxt[:, 1] - prev_end[1])))
            idx = (idx + int(25.0 / ds)) % len(nxt)
            ring = np.vstack([nxt[idx:], nxt[1:idx + 1]])
        t, word = turn(tuple(prev_end[:3]), tuple(ring[0][:3]))
        pieces.append((t[1:], None, f"transition:{word}"))
        pieces.append((ring, np.ones(len(ring), bool), f"headland:{k + 1}"))
        prev_end = ring[-1]

    rows, kinds = [], []
    for sid, (arr, work, kind) in enumerate(pieces):
        wk = work.astype(float) if work is not None else np.zeros(len(arr))
        rows.append(np.column_stack([arr[:, :4], wk, np.zeros(len(arr)), np.full(len(arr), sid)]))
        kinds.append(kind)
    path = np.vstack(rows)
    v_work = im["working_speed_kmh"] / 3.6
    v_turn = field["turn_speed_kmh"] / 3.6
    path[:, 5] = speed_profile(path, v_work, v_turn, a_up=0.4, a_down=0.6)
    # start pose: a short straight run-up behind the first swath's start
    return {"path": path, "kinds": kinds, "angle_deg": deg, "n_swaths": n, "inner_area": A, "field": F,
            "swath_ys": ys, "work_back": work_back, "headland_m": H, "width": w, "radius": R}


def plan_field_fit(field, kit, geom, max_passes=6):
    """Like a farmer with a bigger tractor: widen the headland one pass at a time until every turn stays inside
    the geofence with 0.3 m to spare for steering error."""
    margin = kit["safety"]["geofence_margin_m"]
    for passes in range(field["headland_passes"], max_passes + 1):
        plan = plan_field(dict(field, headland_passes=passes), kit, geom)
        chk = check_plan(plan, geom, kit, margin)
        plan["headland_passes"] = passes
        if chk["max_outside_m"] <= margin - 0.3:
            break
    return plan, chk


def check_plan(plan, geom, kit, margin):
    """Max distance the tractor+planter footprint goes outside the field edge (m), and the minimum turn radius."""
    F = plan["field"]
    p = plan["path"]
    worst, where = 0.0, None
    for i in range(0, len(p), 4):
        fp = footprint(p[i, 0], p[i, 1], p[i, 2], geom, kit)
        if not F.contains(fp):
            d = max(Point(q).distance(F) for part in getattr(fp, "geoms", [fp]) for q in part.exterior.coords)
            if d > worst:
                worst, where = d, plan["kinds"][int(p[i, 6])]
    kmax = float(np.max(np.abs(p[:, 3])))
    return {"max_outside_m": worst, "worst_where": where, "ok": worst <= margin,
            "min_radius_m": 1.0 / kmax if kmax > 0 else float("inf"),
            "length_m": float(_arc_lengths(p[:, :2])[-1]), "points": len(p)}
