"""Render a sim run (results/<run>/log.csv from Gazebo) as a video with the real tractor + planter models.

  blender -b --factory-startup -P blender/replay.py -- --run results/cow_mf_5m_145 --cam chase --speed 1 --seconds 30
  blender ... -- --run results/full_field_mf_5m_145 --cam overview --seconds 40      (speed picked to fit)

Writes PNG frames to blender/out/frames_<run>/ and a HUD subtitle file (hud.ass); tools/encode_video.sh makes the mp4.
"""
import csv
import json
import math
import sys
from pathlib import Path

import bmesh
import bpy
import numpy as np
from mathutils import Vector

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import jbj3d as J  # noqa: E402

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
opt = {argv[i].lstrip("-"): argv[i + 1] for i in range(0, len(argv) - 1, 2)}
RUN = (J.ROOT / opt["run"]).resolve()
CAM = opt.get("cam", "chase")
FPS = int(opt.get("fps", 25))
SECONDS = float(opt.get("seconds", 30))
T0 = float(opt.get("t0", 0))
ONLY = opt.get("frames")            # e.g. "1,200,400" for a quick look
RES = tuple(int(v) for v in opt.get("res", "1280x720").split("x"))
OUT = HERE / "out" / f"frames_{RUN.name}"


def read_log():
    rows = list(csv.DictReader(open(RUN / "log.csv")))

    def col(k):
        return np.array([float(r[k]) if r[k] not in ("", "nan", "None") else np.nan for r in rows])
    d = {k: col(k) for k in ("t", "gt_x", "gt_y", "gt_yaw", "gt_v", "steer", "planter_down", "xte_true", "obstacle_x", "obstacle_y")}
    d["state"] = [r["state"] for r in rows]
    d["gt_yaw"] = np.unwrap(d["gt_yaw"])
    return d


def material(name, rgb, rough=0.9):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*rgb, 1)
    p.inputs["Roughness"].default_value = rough
    return m


def poly_mesh(name, pts, z, mat):
    me = bpy.data.meshes.new(name)
    me.from_pydata([(x, y, z) for x, y in pts], [], [list(range(len(pts)))])
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    me.materials.append(mat)
    return ob


def build_world(field, coll):
    scene = bpy.context.scene
    veld = material("veld", (0.26, 0.25, 0.09))
    soil = material("tilled_soil", (0.16, 0.065, 0.03))
    poly_mesh("veld", [(-600, -600), (800, -600), (800, 700), (-600, 700)], -0.01, veld)
    b = field["boundary_enu_m"]
    poly_mesh("field", b, 0.0, soil)
    cx, cy = sum(p[0] for p in b) / len(b), sum(p[1] for p in b) / len(b)
    post = material("post", (0.25, 0.17, 0.1))
    for i in range(len(b)):
        (x0, y0), (x1, y1) = b[i], b[(i + 1) % len(b)]
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) // 10))
        for k in range(n):
            x, y = x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n
            d = math.hypot(x - cx, y - cy)
            x, y = x + (x - cx) / d * 1.5, y + (y - cy) / d * 1.5
            bpy.ops.mesh.primitive_cylinder_add(vertices=8, radius=0.06, depth=1.2, location=(x, y, 0.6))
            bpy.context.object.data.materials.append(post)
    # flat-topped acacias around the land (typical of the North West bushveld)
    rng = np.random.default_rng(3)
    trunk, crown = material("trunk", (0.18, 0.12, 0.08)), material("crown", (0.13, 0.20, 0.06))
    for _ in range(26):
        ang = rng.uniform(0, 2 * math.pi)
        r = rng.uniform(150, 260)
        x, y = cx + r * math.cos(ang), cy + r * math.sin(ang) * 0.8
        h = rng.uniform(4, 7)
        bpy.ops.mesh.primitive_cylinder_add(vertices=8, radius=0.18, depth=h, location=(x, y, h / 2))
        bpy.context.object.data.materials.append(trunk)
        bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=1.0, location=(x, y, h))
        o = bpy.context.object
        o.scale = (rng.uniform(3, 5), rng.uniform(3, 5), 0.8)
        o.data.materials.append(crown)
    world = bpy.data.worlds.new("sky")
    scene.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs["Color"].default_value = (0.50, 0.66, 0.88, 1)
    world.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.9
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.rotation_euler = (math.radians(45), math.radians(8), math.radians(130))
    scene.collection.objects.link(sun)
    return cx, cy


def planted_rows(log, kit, frame_of):
    """One mesh per planting run: 6 row strips (opener positions), revealed over time with a Build modifier."""
    im = kit["implement"]
    n, sp = im["rows"], im["row_spacing_m"]
    offs = [-(n - 1) * sp / 2 + i * sp for i in range(n)]
    back = 0.95 + 0.85
    mat = material("seed_row", (0.12, 0.05, 0.02))
    work = (log["planter_down"] > 0.5) & (log["gt_v"] > 0.05)
    runs, cur = [], []
    for i in range(len(work)):
        if work[i]:
            cur.append(i)
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    for r_i, idx in enumerate(runs):
        if len(idx) < 3:
            continue
        bm = bmesh.new()
        prev = None
        for i in idx:
            x, y, yaw = log["gt_x"][i], log["gt_y"][i], log["gt_yaw"][i]
            wx, wy = x - back * math.cos(yaw), y - back * math.sin(yaw)
            nx, ny = -math.sin(yaw), math.cos(yaw)
            cur_pts = []
            for o in offs:
                px, py = wx + nx * o, wy + ny * o
                cur_pts.append((bm.verts.new((px + nx * 0.09, py + ny * 0.09, 0.012)), bm.verts.new((px - nx * 0.09, py - ny * 0.09, 0.012))))
            if prev:
                for (a1, a2), (b1, b2) in zip(prev, cur_pts):
                    bm.faces.new((a1, a2, b2, b1))
            prev = cur_pts
        me = bpy.data.meshes.new(f"rows_{r_i}")
        bm.to_mesh(me)
        bm.free()
        ob = bpy.data.objects.new(f"rows_{r_i}", me)
        bpy.context.scene.collection.objects.link(ob)
        me.materials.append(mat)
        f0, f1 = frame_of(log["t"][idx[0]]), frame_of(log["t"][idx[-1]])
        mod = ob.modifiers.new("build", "BUILD")
        mod.frame_start = f0
        mod.frame_duration = max(1, f1 - f0)


def main():
    tractors, kit, field = J.load_specs()
    meta = json.loads((RUN / "plan.json").read_text())
    tid = meta["tractor"]
    t = tractors["models"][tid]
    log = read_log()
    J.clear_scene()
    for ob in list(bpy.data.objects):
        bpy.data.objects.remove(ob)
    scene = bpy.context.scene
    scene.render.fps = FPS
    t_end = log["t"][-1]
    speed = float(opt["speed"]) if "speed" in opt else max(1.0, (t_end - T0) / SECONDS)
    nframes = int(min(SECONDS, (t_end - T0) / speed) * FPS)
    scene.frame_start, scene.frame_end = 1, nframes

    def frame_of(ts):
        return int(round((ts - T0) / speed * FPS)) + 1

    coll = J.new_collection("tractor")
    body, kitobj, g = J.tractor_body(tid, t, kit, coll, J.track_of(t, tractors))
    wf = J.wheel("wf", coll, t["tyre_front"], 20, rear=False)
    wr = J.wheel("wr", coll, t["tyre_rear"], 22, rear=True)
    pl = J.planter(coll, kit)
    kitobj.parent = body
    pl.parent = body
    tr = g["track"]
    rr, rf = g["rear"]["d"] / 2, g["front"]["d"] / 2
    wheels = {}
    for name, src, x, r in (("RL", wr, 0.0, rr), ("RR", wr, 0.0, rr), ("FL", wf, g["L"], rf), ("FR", wf, g["L"], rf)):
        ob = src.copy() if name in ("RR", "FR") else src
        if ob is not src:
            coll.objects.link(ob)
        y = tr / 2 if name.endswith("L") else -tr / 2
        if name.startswith("F"):
            piv = bpy.data.objects.new(f"pivot_{name}", None)
            coll.objects.link(piv)
            piv.parent = body
            piv.location = (x, y, r)
            ob.parent = piv
            ob.location = (0, 0, 0)
            wheels[name] = (ob, piv, r)
        else:
            ob.parent = body
            ob.location = (x, y, r)
            wheels[name] = (ob, None, r)
    cx, cy = build_world(field, coll)
    planted_rows(log, kit, frame_of)
    obst = None
    if np.any(np.isfinite(log["obstacle_x"])):
        oc = J.new_collection("obst")
        kind = "cow" if "cow" in RUN.name else "person"
        obst = J.cow(oc) if kind == "cow" else J.person(oc)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam)
    scene.camera = cam
    cam.data.lens = 35 if CAM == "chase" else 28
    cam.data.clip_end = 2000
    tt = np.array([T0 + (f - 1) * speed / FPS for f in range(1, nframes + 1)])
    X, Y = np.interp(tt, log["t"], log["gt_x"]), np.interp(tt, log["t"], log["gt_y"])
    YAW = np.interp(tt, log["t"], log["gt_yaw"])
    ST = np.interp(tt, log["t"], np.nan_to_num(log["steer"]))
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(X), np.diff(Y)))])
    cam_pos = None
    for f in range(1, nframes + 1):
        k = f - 1
        body.location = (X[k], Y[k], 0)
        body.rotation_euler = (0, 0, YAW[k])
        body.keyframe_insert("location", frame=f)
        body.keyframe_insert("rotation_euler", frame=f)
        for name, (ob, piv, r) in wheels.items():
            ob.rotation_euler = (0, dist[k] / r, 0)
            ob.keyframe_insert("rotation_euler", frame=f)
            if piv is not None:
                piv.rotation_euler = (0, 0, ST[k])
                piv.keyframe_insert("rotation_euler", frame=f)
        if obst is not None:
            ox = np.interp(tt[k], log["t"], np.nan_to_num(log["obstacle_x"], nan=-1e4))
            oy = np.interp(tt[k], log["t"], np.nan_to_num(log["obstacle_y"], nan=-1e4))
            visible = ox > -1e3
            obst.location = (ox, oy, 0) if visible else (0, 0, -50)
            i_near = int(np.searchsorted(log["t"], tt[k]))
            obst.rotation_euler = (0, 0, YAW[k] + math.pi / 2)
            obst.keyframe_insert("location", frame=f)
            obst.keyframe_insert("rotation_euler", frame=f)
        if CAM in ("chase", "behind"):
            c, s = math.cos(YAW[k]), math.sin(YAW[k])
            if CAM == "chase":
                target = Vector((X[k] + 5 * c, Y[k] + 5 * s, 1.0))
                want = Vector((X[k] - 13 * c - 7 * s, Y[k] - 13 * s + 7 * c, 7.0))
            else:      # straight behind and high: shows what is in the path ahead
                target = Vector((X[k] + 12 * c, Y[k] + 12 * s, 0.0))
                want = Vector((X[k] - 15 * c, Y[k] - 15 * s, 10.0))
            cam_pos = want if cam_pos is None else cam_pos.lerp(want, 0.08)
            cam.location = cam_pos
            cam.rotation_euler = (target - cam_pos).to_track_quat("-Z", "Y").to_euler()
        else:
            cam.location = (cx + 150, cy - 150, 120)
            cam.rotation_euler = (Vector((cx, cy + 5, 0)) - Vector(cam.location)).to_track_quat("-Z", "Y").to_euler()
        cam.keyframe_insert("location", frame=f)
        cam.keyframe_insert("rotation_euler", frame=f)
    for eng in ("BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"):
        try:
            scene.render.engine = eng
            break
        except TypeError:
            continue
    scene.render.resolution_x, scene.render.resolution_y = RES
    scene.view_settings.view_transform = "AgX"
    scene.render.image_settings.file_format = "PNG"
    try:
        scene.eevee.taa_render_samples = int(opt.get("samples", 16))   # 16 is plenty for a moving camera
    except AttributeError:
        pass
    OUT.mkdir(parents=True, exist_ok=True)
    write_hud(log, tt, t, kit, speed)
    frames = [int(v) for v in ONLY.split(",")] if ONLY else range(1, nframes + 1)
    for f in frames:
        scene.frame_set(f)
        scene.render.filepath = str(OUT / f"{f:05d}.png")
        bpy.ops.render.render(write_still=True)
    print(f"[jbj] rendered {len(list(frames))} frames of {nframes} ({speed:.1f}x) to {OUT}")


def write_hud(log, tt, t, kit, speed):
    def ts(sec):
        return f"{int(sec // 3600)}:{int(sec % 3600 // 60):02d}:{sec % 60:05.2f}"
    names = {"RUN": "AUTO - planting", "SLOW": "AUTO - slowing for object", "STOP_OBSTACLE": "STOPPED - object in path",
             "STOP_ESTOP": "STOPPED - remote e-stop", "STOP_REMOTE_LOST": "STOPPED - remote signal lost",
             "STOP_RTK": "STOPPED - RTK fix lost", "STOP_OFFTRACK": "STOPPED - off track", "STOP_GEOFENCE": "STOPPED - field edge",
             "DONE": "DONE", "WAIT": "starting"}
    lines = ["[Script Info]", "ScriptType: v4.00+", f"PlayResX: {RES[0]}", f"PlayResY: {RES[1]}", "",
             "[V4+ Styles]", "Format: Name, Fontname, Fontsize, PrimaryColour, OutlineColour, BackColour, Bold, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV",
             "Style: hud,DejaVu Sans,22,&H00FFFFFF,&H00000000,&H90000000,0,3,1,0,7,24,24,20",
             "Style: stop,DejaVu Sans,30,&H004040FF,&H00000000,&H90000000,1,3,1,0,8,24,24,24", "",
             "[Events]", "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text"]
    step = max(1, int(FPS / 5))
    for k in range(0, len(tt), step):
        i = min(len(log["t"]) - 1, int(np.searchsorted(log["t"], tt[k])))
        st = log["state"][i] or ""
        a, b = ts(k / FPS), ts(min(len(tt), k + step) / FPS)
        v = log["gt_v"][i] * 3.6
        x = abs(log["xte_true"][i]) * 100
        planter = "down" if log["planter_down"][i] > 0.5 else "up"
        txt = (f"{t['name']} + {kit['implement']['name']}\\N"
               f"sim time {int(log['t'][i] // 60):d}:{int(log['t'][i] % 60):02d}  ({speed:.0f}x)   speed {v:4.1f} km/h   planter {planter}\\N"
               f"off the line {x:4.1f} cm   mode {names.get(st, st)}")
        lines.append(f"Dialogue: 0,{a},{b},hud,,0,0,0,,{txt}")
        if st.startswith("STOP_") or st == "SLOW":
            lines.append(f"Dialogue: 1,{a},{b},stop,,0,0,0,,{names.get(st, st)}")
    (OUT / "hud.ass").write_text("\n".join(lines), encoding="utf-8")


main()
