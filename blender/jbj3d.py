"""Procedural 3D models for the JBJ autonomous-tractor sim, built from specs/*.json.

Frame (same as ROS/URDF): origin on the ground under the rear-axle centre, X forward, Y left, Z up, metres.
Dimensions that MF publishes (wheelbase, tyres, roof height, length, clearance) come from specs/tractors.json.
Everything else (bonnet shape, cab glass, hitch links) is proportioned to look right: visual only, not used by the sim.
No OEM logos are modelled.
"""
import math
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "jbj_sim"))
from jbj_sim.spec import load, tractor_geometry, tyre_dims  # noqa: E402  (shared with the sim)

COL = {  # base colour, roughness, metallic, alpha, emission
    "mf_red": ((0.60, 0.02, 0.03), 0.35, 0.0, 1.0, 0.0),
    "graphite": ((0.035, 0.035, 0.04), 0.55, 0.2, 1.0, 0.0),
    "rim_grey": ((0.45, 0.46, 0.47), 0.4, 0.5, 1.0, 0.0),
    "tyre": ((0.012, 0.012, 0.012), 0.85, 0.0, 1.0, 0.0),
    "glass": ((0.05, 0.08, 0.10), 0.05, 0.0, 0.35, 0.0),
    "black": ((0.01, 0.01, 0.01), 0.5, 0.0, 1.0, 0.0),
    "chrome": ((0.6, 0.6, 0.6), 0.25, 1.0, 1.0, 0.0),
    "white": ((0.85, 0.85, 0.85), 0.4, 0.0, 1.0, 0.0),
    "estop_red": ((0.75, 0.01, 0.01), 0.3, 0.0, 1.0, 0.0),
    "estop_yellow": ((0.9, 0.65, 0.0), 0.4, 0.0, 1.0, 0.0),
    "amber": ((1.0, 0.45, 0.0), 0.3, 0.0, 1.0, 2.0),
    "lidar": ((0.08, 0.08, 0.09), 0.4, 0.3, 1.0, 0.0),
    "kv_red": ((0.55, 0.02, 0.02), 0.4, 0.0, 1.0, 0.0),
    "kv_grey": ((0.08, 0.08, 0.09), 0.5, 0.3, 1.0, 0.0),
    "seat": ((0.03, 0.03, 0.03), 0.8, 0.0, 1.0, 0.0),
    "cow_brown": ((0.18, 0.08, 0.03), 0.8, 0.0, 1.0, 0.0),
    "cow_white": ((0.75, 0.72, 0.68), 0.8, 0.0, 1.0, 0.0),
    "hivis": ((1.0, 0.35, 0.0), 0.6, 0.0, 1.0, 0.0),
    "denim": ((0.03, 0.05, 0.12), 0.8, 0.0, 1.0, 0.0),
    "skin": ((0.25, 0.13, 0.08), 0.6, 0.0, 1.0, 0.0),
}


def load_specs():
    return load("tractors", ROOT / "specs"), load("kit", ROOT / "specs"), load("field", ROOT / "specs")


# ---------------------------------------------------------------- low-level mesh helpers
_MATS = {}


def mat(name):
    if name in _MATS:
        return _MATS[name]
    rgb, rough, metal, alpha, emit = COL[name]
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes.get("Principled BSDF")
    p.inputs["Base Color"].default_value = (*rgb, 1.0)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    p.inputs["Alpha"].default_value = alpha
    if emit > 0:
        p.inputs["Emission Color"].default_value = (*rgb, 1.0)
        p.inputs["Emission Strength"].default_value = emit
    if alpha < 1.0:
        try:
            m.surface_render_method = "BLENDED"
        except AttributeError:
            m.blend_method = "BLEND"
    m.diffuse_color = (*rgb, alpha)
    _MATS[name] = m
    return m


def _obj(name, bm, material, coll, smooth_quads=False):
    if smooth_quads:
        for f in bm.faces:
            f.smooth = len(f.verts) == 4
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    me.materials.append(mat(material))
    return ob


def TRS(loc=(0, 0, 0), rot=None, scale=(1, 1, 1)):
    R = rot if rot is not None else Matrix.Identity(4)
    return Matrix.Translation(Vector(loc)) @ R @ Matrix.Diagonal((*scale, 1.0))


def rot_x(deg):
    return Matrix.Rotation(math.radians(deg), 4, "X")


def rot_y(deg):
    return Matrix.Rotation(math.radians(deg), 4, "Y")


def rot_z(deg):
    return Matrix.Rotation(math.radians(deg), 4, "Z")


def box(name, material, coll, center, size, rot=None):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0, matrix=TRS(center, rot, size))
    return _obj(name, bm, material, coll)


def cyl(name, material, coll, center, r, h, axis="Z", seg=32, r2=None, rot=None):
    bm = bmesh.new()
    R = {"Z": Matrix.Identity(4), "Y": rot_x(90), "X": rot_y(90)}[axis]
    if rot is not None:
        R = rot @ R
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=seg, radius1=r,
                          radius2=r if r2 is None else r2, depth=h, matrix=TRS(center, R))
    return _obj(name, bm, material, coll, smooth_quads=True)


def ellipsoid(name, material, coll, center, radii, rot=None, seg=24):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=seg, v_segments=seg // 2, radius=1.0, matrix=TRS(center, rot, radii))
    for f in bm.faces:
        f.smooth = True
    return _obj(name, bm, material, coll)


def tapered_box(name, material, coll, x0, x1, y_half, z0, z1, top_scale=0.88, nose_drop=0.12):
    """Bonnet-like box: narrower on top, front top edge rounded down."""
    bm = bmesh.new()
    yt = y_half * top_scale
    v = [bm.verts.new(p) for p in [
        (x0, -y_half, z0), (x1, -y_half, z0), (x1, y_half, z0), (x0, y_half, z0),
        (x0, -yt, z1), (x1 - nose_drop, -yt, z1), (x1 - nose_drop, yt, z1), (x0, yt, z1),
        (x1, -y_half * 0.96, z1 - nose_drop), (x1, y_half * 0.96, z1 - nose_drop)]]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 8, 5, 4), (3, 7, 6, 9, 2), (0, 4, 7, 3), (1, 2, 9, 8), (8, 9, 6, 5)]
    for f in faces:
        bm.faces.new([v[i] for i in f])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _obj(name, bm, material, coll)


def lathe(name, material, coll, profile, seg=48, center=(0, 0, 0)):
    """Revolve (radius, axial) profile points around the Y axis (wheel axis)."""
    bm = bmesh.new()
    rings = []
    for i in range(seg):
        a = 2 * math.pi * i / seg
        rings.append([bm.verts.new((center[0] + r * math.cos(a), center[1] + y, center[2] + r * math.sin(a)))
                      for r, y in profile])
    for i in range(seg):
        A, B = rings[i], rings[(i + 1) % seg]
        for j in range(len(profile) - 1):
            f = bm.faces.new((A[j], A[j + 1], B[j + 1], B[j]))
            f.smooth = True
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _obj(name, bm, material, coll)


def arc_shell(name, material, coll, center, r_in, r_out, width, a0, a1, seg=24):
    """Mudguard: part of a thick cylinder shell around the Y axis, angles in degrees from +X towards +Z."""
    bm = bmesh.new()
    cx, cy, cz = center
    pts = []
    for i in range(seg + 1):
        a = math.radians(a0 + (a1 - a0) * i / seg)
        ring = []
        for r in (r_in, r_out):
            for y in (-width / 2, width / 2):
                ring.append(bm.verts.new((cx + r * math.cos(a), cy + y, cz + r * math.sin(a))))
        pts.append(ring)   # [in-, in+, out-, out+]
    for i in range(seg):
        a, b = pts[i], pts[i + 1]
        bm.faces.new((a[2], a[3], b[3], b[2]))   # outer
        bm.faces.new((a[0], b[0], b[1], a[1]))   # inner
        bm.faces.new((a[0], a[2], b[2], b[0]))   # side -
        bm.faces.new((a[1], b[1], b[3], a[3]))   # side +
    bm.faces.new((pts[0][0], pts[0][1], pts[0][3], pts[0][2]))
    bm.faces.new((pts[-1][0], pts[-1][2], pts[-1][3], pts[-1][1]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return _obj(name, bm, material, coll)


def join(objs, name):
    """Join objects (keeps per-object materials)."""
    objs = [o for o in objs if o is not None]
    if len(objs) == 1:
        objs[0].name = name
        return objs[0]
    ctx = {"active_object": objs[0], "selected_editable_objects": objs, "selected_objects": objs}
    with bpy.context.temp_override(**ctx):
        bpy.ops.object.join()
    objs[0].name = name
    return objs[0]


# ---------------------------------------------------------------- parts
def wheel(prefix, coll, size, pitches, rear=True):
    """Tyre + rim centred at the origin, axis along Y. Chevron lugs point the same way on both sides."""
    td = tyre_dims(size)
    r, w, rr = td["d"] / 2, td["w"], td["rim_d"] / 2
    lug_h = 0.04 if rear else 0.028
    core = r - lug_h
    parts = [lathe(prefix + "_tyre", "tyre", coll, [
        (rr, -w * 0.42), (rr + 0.03, -w * 0.47), (core - 0.06, -w * 0.5), (core - 0.015, -w * 0.495),
        (core, -w * 0.46), (core, w * 0.46), (core - 0.015, w * 0.495), (core - 0.06, w * 0.5),
        (rr + 0.03, w * 0.47), (rr, w * 0.42)])]
    bm = bmesh.new()
    ang = math.radians(45 if rear else 35)
    for i in range(pitches):
        for side in (1, -1):
            length = (w * 0.5) / math.sin(ang) * 0.86     # lug ends stay on the flat part of the tread
            half = length / 2
            # centre the straight lug on the curved tread so both ends sit on the tyre (no sticking-out tips)
            th = 2 * math.pi * (i + (0.5 if side < 0 else 0.0)) / pitches + half * math.cos(ang) / core
            u = Vector((math.cos(th), 0, math.sin(th)))
            tdir = Vector((-math.sin(th), 0, math.cos(th)))
            a = Vector((0, side, 0))
            d = (tdir * math.cos(ang) + a * math.sin(ang)).normalized()
            n = u.cross(d).normalized()
            sag = core - math.sqrt(max(core * core - (half * math.cos(ang)) ** 2, 0.0))
            pos = u * (core + lug_h / 2 - sag / 2) + a * (half * math.sin(ang))
            M = Matrix((
                (d.x, n.x, u.x, pos.x), (d.y, n.y, u.y, pos.y), (d.z, n.z, u.z, pos.z), (0, 0, 0, 1)))
            bmesh.ops.create_cube(bm, size=1.0, matrix=M @ Matrix.Diagonal((length, 0.06 if rear else 0.05, lug_h, 1)))
    parts.append(_obj(prefix + "_lugs", bm, "tyre", coll))
    parts.append(cyl(prefix + "_rim", "rim_grey", coll, (0, 0, 0), rr * 0.99, w * 0.84, axis="Y", seg=40))
    parts.append(cyl(prefix + "_disc", "rim_grey", coll, (0, w * 0.05, 0), rr * 0.7, w * 0.2, axis="Y", seg=32))
    parts.append(cyl(prefix + "_hub", "graphite", coll, (0, w * 0.18, 0), 0.13, 0.22, axis="Y", seg=20))
    return join(parts, prefix)


def tractor_body(tid, t, kit, coll, track):
    """Chassis, bonnet, cab, mudguards, hitch and the retrofit kit, in the tractor frame."""
    g = tractor_geometry(t, track)
    L, rr, fr = g["L"], g["rear"], g["front"]
    r_r, r_f = rr["d"] / 2, fr["d"] / 2
    xf, H = g["x_front"], g["h_roof"]
    inner = track / 2 - rr["w"] / 2           # y of the inner face of the rear tyres
    o = []
    # running gear
    o.append(box("rear_axle_housing", "graphite", coll, (0, 0, r_r), (0.75, 2 * inner - 0.06, 0.55)))
    o.append(cyl("rear_axle_tubes", "graphite", coll, (0, 0, r_r), 0.11, track - 0.2, axis="Y"))
    o.append(box("spine", "graphite", coll, ((0.25 + L + 0.45) / 2, 0, max(g["clearance"], 0.42) + 0.3),
                 (L + 0.45 - 0.25, 0.62, 0.6)))
    o.append(box("front_axle", "graphite", coll, (L, 0, r_f), (0.22, track - fr["w"] - 0.12, 0.2)))
    o.append(box("front_weight_frame", "graphite", coll, ((L + 0.35 + xf) / 2, 0, 0.82), (xf - L - 0.35, 0.72, 0.42)))
    o.append(box("front_bumper", "graphite", coll, (xf - 0.04, 0, 0.82), (0.08, 0.86, 0.36)))
    # bonnet + grille
    bon_top = 0.615 * H
    o.append(tapered_box("bonnet", "mf_red", coll, 0.78, L + 0.62, 0.43, 1.0, bon_top))
    o.append(box("grille", "black", coll, (L + 0.625, 0, (1.0 + bon_top - 0.12) / 2 + 0.02), (0.02, 0.78, bon_top - 1.14)))
    o.append(box("bonnet_lower", "graphite", coll, ((0.78 + L + 0.6) / 2, 0, 0.95), (L + 0.6 - 0.78, 0.8, 0.12)))
    # cab: narrow lower part between the rear tyres, full width above them
    floor = r_r + 0.18
    tyre_top = rr["d"] + 0.06
    xc0, xc1 = -0.72, 0.80
    o.append(box("cab_lower", "graphite", coll, ((xc0 + xc1) / 2, 0, (floor + tyre_top) / 2),
                 (xc1 - xc0, 2 * inner - 0.1, tyre_top - floor)))
    cab_w = 1.36
    z_roof0 = H - 0.14
    for xp in (xc0, xc1):
        for yp in (-cab_w / 2, cab_w / 2):
            zb = floor if abs(yp) < inner else tyre_top
            o.append(box("pillar", "black", coll, (xp, yp, (zb + z_roof0) / 2), (0.06, 0.06, z_roof0 - zb)))
    o.append(box("glass_front", "glass", coll, (xc1, 0, (bon_top + z_roof0) / 2), (0.012, cab_w - 0.06, z_roof0 - bon_top)))
    o.append(box("glass_rear", "glass", coll, (xc0, 0, (tyre_top + z_roof0) / 2), (0.012, cab_w - 0.06, z_roof0 - tyre_top)))
    for s in (-1, 1):
        o.append(box("glass_side", "glass", coll, ((xc0 + xc1) / 2, s * cab_w / 2, (tyre_top + z_roof0) / 2),
                     (xc1 - xc0 - 0.06, 0.012, z_roof0 - tyre_top)))
        o.append(box("cab_sill", "black", coll, ((xc0 + xc1) / 2, s * cab_w / 2, tyre_top), (xc1 - xc0, 0.07, 0.07)))
    o.append(box("roof", "mf_red", coll, ((xc0 + xc1) / 2 - 0.02, 0, H - 0.07), (xc1 - xc0 + 0.22, cab_w + 0.16, 0.14)))
    o.append(box("roof_cap", "white", coll, ((xc0 + xc1) / 2 - 0.02, 0, H - 0.005), (xc1 - xc0 + 0.05, cab_w - 0.04, 0.012)))
    # mudguards over the rear tyres (on the cab sides)
    for s in (-1, 1):
        o.append(arc_shell("mudguard", "mf_red", coll, (0, s * track / 2, r_r), r_r + 0.05, r_r + 0.09,
                           rr["w"] + 0.12, 10, 170, seg=28))
    # exhaust on the right-hand front cab pillar
    o.append(cyl("exhaust", "graphite", coll, (xc1 + 0.06, -0.42, (bon_top + H + 0.15) / 2), 0.055, H + 0.15 - bon_top))
    # interior: seat, console
    o.append(box("seat", "seat", coll, (-0.2, 0, floor + 0.5), (0.5, 0.5, 0.12)))
    o.append(box("seat_back", "seat", coll, (-0.45, 0, floor + 0.85), (0.1, 0.5, 0.6)))
    o.append(box("console_r", "graphite", coll, (-0.1, -0.42, floor + 0.62), (0.6, 0.18, 0.25)))
    # three-point hitch (ISO 730 Cat 2: lower hitch points 825 mm apart, mast height 610 mm)
    zl = 0.45
    for s in (-1, 1):
        a, b = Vector((-0.30, s * 0.40, 0.55)), Vector((-0.95, s * 0.4125, zl))
        d = b - a
        o.append(cyl("lower_link", "graphite", coll, (a + b) / 2, 0.035, d.length, axis="Z",
                     rot=d.to_track_quat("Z", "Y").to_matrix().to_4x4()))
        o.append(box("lift_arm", "graphite", coll, (-0.45, s * 0.35, r_r + 0.35), (0.5, 0.06, 0.08), rot=rot_y(12)))
        o.append(cyl("lift_rod", "graphite", coll, (-0.65, s * 0.38, (zl + r_r + 0.3) / 2 + 0.05), 0.022, r_r + 0.3 - zl - 0.1))
    a, b = Vector((-0.42, 0, zl + 0.61 + 0.05)), Vector((-0.97, 0, zl + 0.61))
    d = b - a
    o.append(cyl("top_link", "graphite", coll, (a + b) / 2, 0.04, d.length, rot=d.to_track_quat("Z", "Y").to_matrix().to_4x4()))
    body = join(o, f"{tid}_body")

    # ---- retrofit kit (kept as its own object so it can be highlighted)
    k = []
    ant_y = kit["gnss"]["antenna_baseline_m"] / 2
    for s in (-1, 1):
        k.append(cyl("gnss_base", "white", coll, (0.30, s * ant_y, H + 0.03), 0.08, 0.06, seg=24))
        k.append(ellipsoid("gnss_dome", "white", coll, (0.30, s * ant_y, H + 0.06), (0.08, 0.08, 0.05)))
    k.append(box("camera", "black", coll, (xc1 + 0.1, 0, H - 0.12), (0.10, 0.09, 0.07)))
    k.append(cyl("beacon", "amber", coll, (-0.55, 0.52, H + 0.075), 0.07, 0.15, seg=24))
    k.append(cyl("lte_antenna", "black", coll, (-0.55, -0.52, H + 0.13), 0.012, 0.26, seg=12))
    # Livox Mid-360 on a short mast at the front, upside down; optical centre at spec.lidar_xyz (1.30 m)
    lx, lz = g["lidar_xyz"][0] - 0.1, g["lidar_xyz"][2]
    k.append(box("lidar_mast", "graphite", coll, (lx, 0, (1.0 + lz + 0.12) / 2), (0.06, 0.06, lz + 0.12 - 1.0)))
    k.append(box("lidar_arm", "graphite", coll, (lx + 0.06, 0, lz + 0.09), (0.14, 0.08, 0.02)))
    k.append(box("lidar_body", "lidar", coll, (lx + 0.1, 0, lz + 0.055), (0.065, 0.065, 0.03)))
    k.append(cyl("lidar_window", "black", coll, (lx + 0.1, 0, lz + 0.01), 0.03, 0.035, seg=24))
    # steering motor (180 x 77 mm) under a 380 mm wheel, terminal, control box
    sw = Vector((0.42, 0, floor + 0.95))
    tilt = rot_y(-35)
    k.append(cyl("steer_motor", "graphite", coll, sw - Vector((0.03, 0, 0.06)), 0.09, 0.077, rot=tilt))
    k.append(cyl("steering_wheel", "black", coll, sw, 0.19, 0.03, rot=tilt, seg=32))
    k.append(box("terminal", "black", coll, (0.66, -0.45, floor + 1.05), (0.03, 0.30, 0.20), rot=rot_z(25)))
    k.append(box("control_box", "rim_grey", coll, (-0.62, -0.38, floor + 0.35), (0.12, 0.30, 0.25)))
    # e-stops: one on each mudguard, one on the front bumper, one in the cab
    stops = [(0.0, track / 2 + rr["w"] / 2 + 0.02, r_r + 0.3), (0.0, -(track / 2 + rr["w"] / 2 + 0.02), r_r + 0.3),
             (xf + 0.02, 0.3, 1.0), (0.1, -0.42, floor + 0.78)]
    for i, (x, y, z) in enumerate(stops):
        k.append(box("estop_box", "estop_yellow", coll, (x, y, z), (0.08, 0.08, 0.08)))
        k.append(cyl("estop_button", "estop_red", coll, (x, y, z + 0.055), 0.032, 0.03, seg=20))
    kitobj = join(k, f"{tid}_kit")
    return body, kitobj, g


def planter(coll, kit):
    """Kverneland Optima V style 6-row precision planter, origin at the tractor's lower hitch points (x=-0.95)."""
    im = kit["implement"]
    n, sp = im["rows"], im["row_spacing_m"]
    o = []
    x0, zl = -0.95, 0.45
    o.append(box("headstock", "kv_grey", coll, (x0 - 0.12, 0, zl + 0.33), (0.12, 0.9, 0.75)))
    o.append(box("toolbar", "kv_grey", coll, (x0 - 0.35, 0, zl + 0.30), (0.15, n * sp + 0.25, 0.15)))
    for i in range(n):
        y = -(n - 1) * sp / 2 + i * sp
        o.append(box("parallelogram", "kv_grey", coll, (x0 - 0.62, y, zl + 0.20), (0.45, 0.08, 0.06), rot=rot_y(-15)))
        o.append(box("hopper", "kv_red", coll, (x0 - 0.95, y, zl + 0.62), (0.42, 0.36, 0.48)))
        o.append(box("hopper_lid", "black", coll, (x0 - 0.95, y, zl + 0.875), (0.44, 0.38, 0.03)))
        o.append(box("unit_frame", "kv_grey", coll, (x0 - 1.05, y, zl + 0.12), (0.7, 0.12, 0.18)))
        for s in (-1, 1):
            o.append(cyl("gauge_wheel", "tyre", coll, (x0 - 0.95, y + s * 0.09, 0.19), 0.19, 0.08, axis="Y", seg=24))
        o.append(cyl("opener_disc", "chrome", coll, (x0 - 0.85, y, 0.17), 0.18, 0.012, axis="Y", seg=24))
        o.append(cyl("closing_wheel", "kv_grey", coll, (x0 - 1.45, y, 0.14), 0.14, 0.05, axis="Y", seg=20, rot=rot_x(12)))
    for s in (-1, 1):   # folded row markers
        yy = s * (n * sp / 2 + 0.15)
        o.append(box("marker_arm", "kv_red", coll, (x0 - 0.35, yy, zl + 1.05), (0.08, 0.08, 1.4)))
        o.append(cyl("marker_disc", "chrome", coll, (x0 - 0.35, yy, zl + 1.8), 0.17, 0.012, axis="X", seg=24))
    return join(o, "implement_planter")


def cow(coll):
    """Adult cow, about 2.1 m long and 1.35 m tall (Nguni/Bonsmara size)."""
    o = [ellipsoid("cow_body", "cow_brown", coll, (0, 0, 1.0), (0.80, 0.30, 0.36)),
         ellipsoid("cow_patch", "cow_white", coll, (0.1, 0, 0.82), (0.55, 0.27, 0.2)),
         ellipsoid("cow_neck", "cow_brown", coll, (0.78, 0, 1.12), (0.25, 0.16, 0.2), rot=rot_y(-30)),
         ellipsoid("cow_head", "cow_brown", coll, (1.02, 0, 1.15), (0.24, 0.12, 0.14), rot=rot_y(35))]
    for x in (0.5, -0.55):
        for y in (0.17, -0.17):
            o.append(cyl("cow_leg", "cow_brown", coll, (x, y, 0.36), 0.06, 0.72, seg=12))
    o.append(cyl("cow_tail", "cow_brown", coll, (-0.82, 0, 0.8), 0.02, 0.55, seg=8, rot=rot_y(15)))
    return join(o, "obstacle_cow")


def person(coll):
    """Adult (1.75 m) in a hi-vis vest."""
    o = [cyl("legs", "denim", coll, (0, 0.1, 0.43), 0.075, 0.86, seg=16), cyl("legs2", "denim", coll, (0, -0.1, 0.43), 0.075, 0.86, seg=16),
         box("torso", "hivis", coll, (0, 0, 1.15), (0.24, 0.42, 0.6)),
         cyl("arm_l", "hivis", coll, (0, 0.27, 1.12), 0.05, 0.6, seg=12), cyl("arm_r", "hivis", coll, (0, -0.27, 1.12), 0.05, 0.6, seg=12),
         ellipsoid("head", "skin", coll, (0, 0, 1.6), (0.1, 0.09, 0.12))]
    return join(o, "obstacle_person")


def new_collection(name, parent=None):
    c = bpy.data.collections.new(name)
    (parent or bpy.context.scene.collection).children.link(c)
    return c


def clear_scene():
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob)
    for c in list(bpy.data.collections):
        bpy.data.collections.remove(c)
    _MATS.clear()
