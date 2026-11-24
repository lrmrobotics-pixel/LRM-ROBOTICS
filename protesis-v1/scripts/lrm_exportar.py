# -*- coding: utf-8 -*-
"""Exporta cada pieza como STL en milimetros (pose ensamblada, coordenadas de mundo) y el conjunto como GLB
con la jerarquia de juntas.   blender -b LRM_Protesis_V1.blend --python scripts/lrm_exportar.py"""
import bpy, os

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(PROJ, "exports", "stl_por_pieza_mm")
os.makedirs(STL_DIR, exist_ok=True)
sc = bpy.context.scene
sc.frame_set(1)
prot = bpy.data.collections["LRM_PROTESIS_V1"]
parts = [o for o in prot.all_objects if o.type == 'MESH']

for o in parts:
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.wm.stl_export(filepath=os.path.join(STL_DIR, o.name + ".stl"), export_selected_objects=True,
                          global_scale=1000.0, apply_modifiers=True, ascii_format=False)
print(f"[EXPORT] {len(parts)} STL en {STL_DIR}")

# GLB: protesis + juntas (sin escenario ni tendones virtuales)
bpy.ops.object.select_all(action='DESELECT')
for o in prot.all_objects:
    if o.type in ('MESH', 'EMPTY'):
        o.select_set(True)
glb = os.path.join(PROJ, "exports", "LRM_Protesis_V1_rig.glb")
bpy.ops.export_scene.gltf(filepath=glb, export_format='GLB', use_selection=True, export_extras=True,
                          export_animations=False, export_yup=True)
print("[EXPORT] GLB", glb)
