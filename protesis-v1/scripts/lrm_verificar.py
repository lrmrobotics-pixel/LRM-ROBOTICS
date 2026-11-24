# -*- coding: utf-8 -*-
"""Verificacion de interferencias del modelo LRM V1.
blender -b LRM_Protesis_V1.blend --python scripts/lrm_verificar.py
1) piezas que se cruzan en pose ensamblada  2) barrido de cada junta en su rango  3) mallas no-manifold"""
import bpy, bmesh, math
from mathutils import Vector
from mathutils.bvhtree import BVHTree

scene = bpy.context.scene
scene.frame_set(1)
prot = bpy.data.collections["LRM_PROTESIS_V1"]
parts = [o for o in prot.all_objects if o.type == 'MESH']
joints = {o.name: o for o in prot.all_objects if o.type == 'EMPTY'}
print(f"[CHK] piezas={len(parts)} juntas={len(joints)}")


GEO = {}


def bvh(o):
    dg = bpy.context.evaluated_depsgraph_get()
    me = o.evaluated_get(dg).to_mesh()
    mw = o.matrix_world
    vs = [mw @ v.co for v in me.vertices]
    ps = [tuple(p.vertices) for p in me.polygons]
    t = BVHTree.FromPolygons(vs, ps, epsilon=0.0)
    GEO[o.name] = (vs, ps)
    o.evaluated_get(dg).to_mesh_clear()
    return t


def bbox(o):
    cs = [o.matrix_world @ Vector(c) for c in o.bound_box]
    return (Vector((min(c.x for c in cs), min(c.y for c in cs), min(c.z for c in cs))),
            Vector((max(c.x for c in cs), max(c.y for c in cs), max(c.z for c in cs))))


def bb_hit(a, b, m=0.0005):
    return all(a[0][i] - m <= b[1][i] and b[0][i] - m <= a[1][i] for i in range(3))


def overlaps(set_a, set_b=None, label=""):
    bpy.context.view_layer.update()
    trees = {o.name: bvh(o) for o in set(set_a) | set(set_b or [])}
    boxes = {o.name: bbox(o) for o in set(set_a) | set(set_b or [])}
    hits = []
    pairs = ([(a, b) for i, a in enumerate(set_a) for b in set_a[i + 1:]] if set_b is None
             else [(a, b) for a in set_a for b in set_b if a != b])
    for a, b in pairs:
        if not bb_hit(boxes[a.name], boxes[b.name]):
            continue
        ov = trees[a.name].overlap(trees[b.name])
        if ov:
            vs, ps = GEO[a.name]
            c = sum((vs[i] for i in ps[ov[0][0]]), Vector()) / len(ps[ov[0][0]])
            hits.append((a.name, b.name, len(ov), "@mm", tuple(round(v * 1000, 1) for v in c)))
    return hits


# 1) reposo
hits = overlaps(parts)
print(f"[CHK] interferencias en reposo: {len(hits)}")
for h in sorted(hits, key=lambda x: -x[2]):
    print("   ", h)


# 2) barridos
def descendants_meshes(j):
    out = []
    for c in j.children_recursive:
        if c.type == 'MESH':
            out.append(c)
    return out


def sweep(jname, channel, angles, label):
    j = joints[jname]
    moving = descendants_meshes(j)
    static = [p for p in parts if p not in moving]
    idx = 'XYZ'.index(channel)
    rest = j.rotation_euler[idx]
    res = []
    for a in angles:
        j.rotation_euler[idx] = math.radians(a)
        h = overlaps(moving, static)
        res.append((a, h))
    j.rotation_euler[idx] = rest
    bpy.context.view_layer.update()
    ok = [a for a, h in res if not h]
    print(f"[CHK] {label}:")
    for a, h in res:
        print(f"     {a:7.1f} deg -> {'OK' if not h else h[:4]}")
    return res


# las juntas tienen limites (constraint) y drivers del panel; para barrer se desactivan
# (al borrar los drivers la rotacion de reposo ya evaluada se conserva)
for j in joints.values():
    j.animation_data_clear()
    for c in j.constraints:
        c.mute = True

sweep("elbow_joint", 'Y', [0, -30, -60, -90, -110, -120, -135], "CODO flexion (0..135, canal Y negativo)")
sweep("wrist_joint", 'X', [-90, -45, 45, 90], "MUNECA prono-supinacion")
for f in ("index", "middle", "ring", "pinky"):
    for jn in ("mcp", "pip", "dip"):
        sweep(f"{f}_{jn}_joint", 'Y', [0, 45, 90], f"{f} {jn}")
sweep("thumb_cmc_opposition_joint", 'X', [-15, -30, -45], "PULGAR oposicion")
sweep("thumb_cmc_flexion_joint", 'Y', [15, 30], "PULGAR CMC flexion")
for jn in ("mcp", "ip"):
    sweep(f"thumb_{jn}_joint", 'Y', [30, 60, 90] if jn == "ip" else [30, 60], f"PULGAR {jn}")

# 3) manifold
bad = []
for o in parts:
    bm = bmesh.new()
    bm.from_mesh(o.data)
    nm = sum(1 for e in bm.edges if not e.is_manifold)
    if nm:
        bad.append((o.name, nm))
    bm.free()
print(f"[CHK] mallas con aristas no-manifold: {len(bad)}")
for b in bad:
    print("    ", b)
