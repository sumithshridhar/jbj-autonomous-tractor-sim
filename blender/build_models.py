"""Build every MF tractor + retrofit kit + planter + obstacles from specs/, save the .blend, export meshes for Gazebo.

  blender -b --factory-startup -P blender/build_models.py [-- --render 1]

Outputs
  blender/out/jbj_models.blend
  ros2_ws/src/jbj_sim/models/<tractor_id>/meshes/{body,kit,wheel_front,wheel_rear}.obj  (tractor frame, metres)
  ros2_ws/src/jbj_sim/models/planter/meshes/planter.obj, models/cow/..., models/person/...
  media/<tractor_id>.png  (with --render 1)
"""
import math
import sys
from pathlib import Path

import bpy

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import jbj3d as J  # noqa: E402
from mathutils import Vector  # noqa: E402

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
opt = {argv[i].lstrip("-"): argv[i + 1] for i in range(0, len(argv) - 1, 2)}
RENDER = opt.get("render", "0") == "1"
ROOT = J.ROOT
MODELS = ROOT / "ros2_ws" / "src" / "jbj_sim" / "models"
MEDIA = ROOT / "media"
OUT = HERE / "out"


def export_obj(objs, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.wm.obj_export(filepath=str(path), export_selected_objects=True, forward_axis="Y", up_axis="Z",
                          export_materials=True, apply_modifiers=True, export_uv=False, export_normals=True,
                          export_triangulated_mesh=True)


def place(ob, loc, rot_z_deg=0.0):
    ob.location = loc
    ob.rotation_euler = (0, 0, math.radians(rot_z_deg))


def main():
    tractors, kit, field = J.load_specs()
    J.clear_scene()
    scene = bpy.context.scene

    built = {}
    for i, (tid, t) in enumerate(tractors["models"].items()):
        coll = J.new_collection(tid)
        track = J.track_of(t, tractors)
        body, kitobj, g = J.tractor_body(tid, t, kit, coll, track)
        wf = J.wheel(f"{tid}_wheel_front", coll, t["tyre_front"], 20, rear=False)
        wr = J.wheel(f"{tid}_wheel_rear", coll, t["tyre_rear"], 22, rear=True)
        # meshes for Gazebo are exported with the wheel at its own origin
        export_obj([body], MODELS / tid / "meshes" / "body.obj")
        export_obj([kitobj], MODELS / tid / "meshes" / "kit.obj")
        export_obj([wf], MODELS / tid / "meshes" / "wheel_front.obj")
        export_obj([wr], MODELS / tid / "meshes" / "wheel_rear.obj")
        # assembled copy for the .blend / renders: parent wheels + kit to the body
        rr, fr = g["rear"]["d"] / 2, g["front"]["d"] / 2
        wheels = []
        for name, src, x, r in (("RL", wr, 0.0, rr), ("RR", wr, 0.0, -rr), ("FL", wf, g["L"], fr), ("FR", wf, g["L"], -fr)):
            ob = src if name in ("RL", "FL") else src.copy()
            if ob is not src:
                coll.objects.link(ob)
            ob.name = f"{tid}_wheel_{name}"
            y = track / 2 if name.endswith("L") else -track / 2
            ob.location = (x, y, abs(r))
            ob.parent = body
            wheels.append(ob)
        kitobj.parent = body
        built[tid] = (body, kitobj, wheels, g)
        place(body, (0, i * 6.0, 0))
    impl_coll = J.new_collection("implement")
    pl = J.planter(impl_coll, kit)
    export_obj([pl], MODELS / "planter" / "meshes" / "planter.obj")
    obst = J.new_collection("obstacles")
    c = J.cow(obst)
    p = J.person(obst)
    export_obj([c], MODELS / "cow" / "meshes" / "cow.obj")
    export_obj([p], MODELS / "person" / "meshes" / "person.obj")
    place(c, (12, -6, 0), 200)
    place(p, (10, -9, 0), 160)
    pl.parent = built[tractors["default"]][0]
    OUT.mkdir(exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT / "jbj_models.blend"))
    for tid, (body, kitobj, wheels, g) in built.items():
        print(f"[jbj] {tid}: L={g['L']:.3f} track={g['track']:.2f} rear_d={g['rear']['d']:.3f} front_d={g['front']['d']:.3f} "
              f"r_min(rear axle)={g['r_min_rear_axle']:.2f} m outer_front_r={g['r_outer_front']:.2f} m "
              f"delta_eq_max={math.degrees(g['delta_eq_max']):.1f} deg x_front={g['x_front']:.2f}")
    if RENDER:
        render_all(built, pl, tractors)


def setup_render(scene, res=(1600, 900)):
    for eng in ("BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"):
        try:
            scene.render.engine = eng
            break
        except TypeError:
            continue
    scene.render.resolution_x, scene.render.resolution_y = res
    scene.render.film_transparent = False
    scene.view_settings.view_transform = "AgX"
    world = bpy.data.worlds.new("sky") if not scene.world else scene.world
    scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = (0.55, 0.68, 0.85, 1)
    bg.inputs["Strength"].default_value = 0.8
    sun = bpy.data.lights.new("sun", "SUN")
    sun.energy = 4.0
    sun.angle = math.radians(3)
    so = bpy.data.objects.new("sun", sun)
    scene.collection.objects.link(so)
    so.rotation_euler = (math.radians(50), math.radians(10), math.radians(140))
    me = bpy.data.meshes.new("ground")
    me.from_pydata([(-200, -200, 0), (200, -200, 0), (200, 200, 0), (-200, 200, 0)], [], [(0, 1, 2, 3)])
    gr = bpy.data.objects.new("ground", me)
    scene.collection.objects.link(gr)
    m = bpy.data.materials.new("soil")
    m.use_nodes = True
    m.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.20, 0.09, 0.04, 1)
    m.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.95
    me.materials.append(m)


def look_at(cam, target):
    d = Vector(target) - cam.location
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()


def render_all(built, planter_obj, tractors):
    scene = bpy.context.scene
    setup_render(scene)
    cam_data = bpy.data.cameras.new("cam")
    cam_data.lens = 50
    cam = bpy.data.objects.new("cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    MEDIA.mkdir(exist_ok=True)
    for tid in built:   # one tractor at a time, at the origin, with the planter on the default one
        for other, (b, k, w, g) in built.items():
            vis = other == tid
            for ob in [b, k, *w]:
                ob.hide_render = not vis
            b.location = (0, 0, 0)
        planter_obj.parent = built[tid][0]
        planter_obj.hide_render = False
        for o in bpy.data.objects:
            if o.name.startswith("obstacle_"):
                o.hide_render = True
        g = built[tid][3]
        s = g["h_roof"] / 2.78                      # keep big tractors in frame
        cam.location = (g["L"] + 6.5 * s, -7.5 * s, 3.2 * s)
        look_at(cam, (g["L"] * 0.35, 0, 1.2 * s))
        scene.render.filepath = str(MEDIA / f"{tid}.png")
        bpy.ops.render.render(write_still=True)
        print(f"[jbj] rendered {scene.render.filepath}")
    # kit close-up on the default tractor
    tid = tractors["default"]
    for other, (b, k, w, g) in built.items():
        for ob in [b, k, *w]:
            ob.hide_render = other != tid
    g = built[tid][3]
    cam_data.lens = 35
    cam.location = (g["x_front"] + 3.2, 2.6, g["h_roof"] + 1.6)
    look_at(cam, (g["x_front"] * 0.45, 0, g["h_roof"] * 0.62))
    scene.render.filepath = str(MEDIA / "kit_closeup.png")
    bpy.ops.render.render(write_still=True)
    # obstacles
    for o in bpy.data.objects:
        if o.name.startswith("obstacle_"):
            o.hide_render = False


main()
