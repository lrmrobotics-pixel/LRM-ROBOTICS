# -*- coding: utf-8 -*-
"""Render de vistas con encuadre automatico.
blender -b LRM_Protesis_V1.blend --python scripts/lrm_render.py -- <vista|all> [preview]"""
import bpy, os, sys, math
from mathutils import Vector

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
which = argv[0] if argv else "all"
preview = "preview" in argv
PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PROJ, "renders")
os.makedirs(OUT, exist_ok=True)
sc = bpy.context.scene
prot = bpy.data.collections["LRM_PROTESIS_V1"]


def coll_meshes(*names):
    out = []
    for n in names:
        out += [o for o in bpy.data.collections[n].all_objects if o.type == 'MESH']
    return out


ALL = [o for o in prot.all_objects if o.type == 'MESH']
HAND = coll_meshes("05_MANO_palma", "06_DEDOS", "07_PULGAR")
SOLAR = [bpy.data.objects[n] for n in ("forearm_main_housing", "solar_panel_dorsal_forearm", "solar_panel_support_base",
                                       "solar_contact_connector", "forearm_lateral_cosmetic_cover",
                                       "forearm_proximal_cap")]

# nombre: (camara, controles de LRM_CONTROL, objetos a encuadrar, direccion de vista, margen)
VIEWS = {
    "ensamblada": ("CAM_ensamblada_3_4", {}, ALL, (-0.32, 1.0, -0.52), 1.06),
    "explosionada": ("CAM_explosionada", {"explosion": 1.0}, ALL, (0.0, 1.0, -1.25), 1.03),
    "mano_explosionada": ("CAM_mano_explosionada", {"explosion": 1.0}, HAND, (0.0, 0.42, -1.0), 1.12),
    "panel_solar": ("CAM_panel_solar", {"panel_solar": 1.0}, SOLAR, (0.45, 1.0, -0.95), 1.08),
}
CTRL = bpy.data.objects["LRM_CONTROL"]
CTRL.animation_data_clear()
CTRL_PROPS = [k for k in CTRL.keys() if k != "descripcion"]


def set_controls(vals):
    for k in CTRL_PROPS:
        CTRL[k] = float(vals.get(k, 0.0))
    CTRL.update_tag()
    sc.frame_set(1)


def fit_camera(cam, objs, direction, margin):
    bpy.context.view_layer.update()
    pts = []
    for o in objs:
        pts += [o.matrix_world @ Vector(c) for c in o.bound_box]
    mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
    mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
    c = (mn + mx) / 2
    f = Vector(direction).normalized()
    right = f.cross(Vector((0, 0, 1))).normalized()
    up = right.cross(f).normalized()
    cd = cam.data
    aspect = sc.render.resolution_x / sc.render.resolution_y
    th = math.tan(math.atan(cd.sensor_width / 2 / cd.lens))
    tv = th / aspect
    D = 0.0
    for p in pts:
        v = p - c
        x, y, z = v.dot(right), v.dot(up), v.dot(f)
        D = max(D, abs(x) / th - z, abs(y) / tv - z)
    D *= margin
    cam.location = c - f * D
    cam.rotation_euler = f.to_track_quat('-Z', 'Y').to_euler()


if preview:
    sc.render.resolution_percentage = 50
    sc.cycles.samples = 40
for name, (cam, ctrl_vals, objs, d, m) in VIEWS.items():
    if which not in ("all", name):
        continue
    set_controls(ctrl_vals)
    co = bpy.data.objects[cam]
    co.data.sensor_fit = 'HORIZONTAL'
    fit_camera(co, objs, d, m)
    sc.camera = co
    sc.render.filepath = os.path.join(OUT, f"LRM_V1_{name}{'_preview' if preview else ''}.png")
    bpy.ops.render.render(write_still=True)
    print("[RENDER]", sc.render.filepath)

if not preview and which == "all":
    set_controls({})
    sc.camera = bpy.data.objects["CAM_ensamblada_3_4"]
    bpy.ops.wm.save_mainfile()
