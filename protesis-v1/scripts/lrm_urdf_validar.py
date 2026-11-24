# -*- coding: utf-8 -*-
"""Valida el URDF contra el modelo de Blender (sin necesitar PyBullet).
blender -b LRM_Protesis_V1.blend --python scripts/lrm_urdf_validar.py
1) estructura: arbol unico, mallas existentes, limites coherentes
2) inercias: definidas positivas y desigualdad triangular
3) cinematica directa del URDF == posicion de las juntas en Blender, en 200 poses aleatorias"""
import bpy, os, math, random
import xml.etree.ElementTree as ET
from mathutils import Matrix, Vector, Euler

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(PROJ, "exports", "urdf", "lrm_protesis_v1.urdf")
root = ET.parse(URDF).getroot()
errs = []

links = {l.get("name"): l for l in root.findall("link")}
joints = {j.get("name"): j for j in root.findall("joint")}
children = {j.find("child").get("link") for j in joints.values()}
bases = [n for n in links if n not in children]
if len(bases) != 1:
    errs.append(f"base no unica: {bases}")
for n, l in links.items():
    for g in l.iter("mesh"):
        f = os.path.join(os.path.dirname(URDF), g.get("filename"))
        if not os.path.isfile(f):
            errs.append(f"malla faltante {f}")
    it = l.find("inertial/inertia")
    I = [float(it.get(k)) for k in ("ixx", "iyy", "izz")]
    M = Matrix(((float(it.get("ixx")), float(it.get("ixy")), float(it.get("ixz"))),
                (float(it.get("ixy")), float(it.get("iyy")), float(it.get("iyz"))),
                (float(it.get("ixz")), float(it.get("iyz")), float(it.get("izz")))))
    if M.determinant() <= 0 or min(I) <= 0:
        errs.append(f"inercia no definida positiva: {n}")
    a, b, c = I
    if a + b < c * 0.999 or a + c < b * 0.999 or b + c < a * 0.999:
        errs.append(f"inercia viola desigualdad triangular: {n}")
for n, j in joints.items():
    lim = j.find("limit")
    if float(lim.get("lower")) >= float(lim.get("upper")):
        errs.append(f"limites invertidos: {n}")
print(f"[VAL] eslabones={len(links)} juntas={len(joints)} base={bases}")


def origin_mat(el):
    o = el.find("origin")
    t = Vector([float(v) for v in o.get("xyz").split()])
    r = [float(v) for v in o.get("rpy").split()]
    return Matrix.Translation(t) @ Euler(r, 'XYZ').to_matrix().to_4x4()


def fk(q):
    W = {bases[0]: Matrix.Identity(4)}
    pending = dict(joints)
    while pending:
        for n, j in list(pending.items()):
            p = j.find("parent").get("link")
            if p in W:
                ax = Vector([float(v) for v in j.find("axis").get("xyz").split()])
                W[j.find("child").get("link")] = W[p] @ origin_mat(j) @ Matrix.Rotation(q.get(n, 0.0), 4, ax)
                del pending[n]
    return W


# Blender: rotacion de los Empties (canal local, signo segun eje)
sc = bpy.context.scene
emp = {o.name: o for o in bpy.data.collections["00_CINEMATICA_juntas"].objects}
for e in emp.values():
    e.animation_data_clear()
    for c in e.constraints:
        c.mute = True
random.seed(7)
worst = 0.0
for trial in range(200):
    q = {}
    for n, j in joints.items():
        lim = j.find("limit")
        q[n] = random.uniform(float(lim.get("lower")), float(lim.get("upper")))
    for n, e in emp.items():
        e.rotation_euler = (0, 0, 0)
        if n in q:
            idx = 'XYZ'.index(e["rot_channel"])
            r = [0.0, 0.0, 0.0]
            r[idx] = q[n] * sum(e["axis_local"])
            e.rotation_euler = r
    bpy.context.view_layer.update()
    W = fk(q)
    for n, M in W.items():
        Mb = emp[n].matrix_world
        d = (M.to_translation() - Mb.to_translation()).length
        ang = math.degrees(M.to_quaternion().rotation_difference(Mb.to_quaternion()).angle)
        ang = min(ang, 360 - ang)
        worst = max(worst, d * 1000 + ang)
        if d > 1e-5 or ang > 0.05:      # Blender guarda matrices en float32
            errs.append(f"FK difiere en {n}: {d * 1000:.3f} mm {ang:.3f} deg")
print(f"[VAL] cinematica directa: 200 poses aleatorias, error maximo {worst:.6f} (mm+deg)")
print(f"[VAL] errores: {len(errs)}")
for e in errs[:20]:
    print("   ", e)
