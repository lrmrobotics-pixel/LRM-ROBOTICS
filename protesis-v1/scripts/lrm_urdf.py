# -*- coding: utf-8 -*-
"""Genera el URDF de la protesis LRM V1 desde el .blend (misma fuente que el JSON cinematico).
blender -b LRM_Protesis_V1.blend --python scripts/lrm_urdf.py

- Un <link> por eslabon cinematico (Empty de 00_CINEMATICA_juntas) con UNA malla STL que une todas
  las piezas de ese eslabon, expresada en el marco del eslabon (metros) con todas las juntas en 0.
- Masa, centro de masa e inercia calculados de la malla real (integracion por tetraedros) con la
  densidad de cada material y un factor de relleno de impresion 3D para piezas impresas.
- Juntas revolute con eje, limites (rad), esfuerzo y velocidad estimados por tipo de actuador.
- Cero de cada junta = dedos rectos (la flexion de reposo de Blender va en <rest_deg> como comentario).
"""
import bpy, bmesh, os, math, json
from mathutils import Matrix, Vector
from xml.sax.saxutils import escape

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(PROJ, "exports", "urdf")
MESH_DIR = os.path.join(OUT, "meshes")
os.makedirs(MESH_DIR, exist_ok=True)

sc = bpy.context.scene
sc.frame_set(1)
prot = bpy.data.collections["LRM_PROTESIS_V1"]
links = [o for o in bpy.data.collections["00_CINEMATICA_juntas"].objects if o.type == 'EMPTY']
parts = [o for o in prot.all_objects if o.type == 'MESH']

# pose cero: todas las juntas en 0, sin explosion
for e in links:
    e.animation_data_clear()
    e.rotation_euler = (0, 0, 0)
for p in parts:
    p.animation_data_clear()
    p.delta_location = (0, 0, 0)
    p.delta_rotation_euler = (0, 0, 0)
bpy.context.view_layer.update()

# densidad efectiva kg/m3 (impresion FDM ~ 60% relleno+paredes para rPET; macizos el resto)
DENS = {"rPET reciclado": 1380 * 0.60, "rPET reforzado EBA-GMA": 1350 * 0.70, "TPU": 1210 * 0.80,
        "Silicona medica platino ISO 10993": 1120, "PET negro": 1380 * 0.80, "Acero inox": 7900,
        "Acero resorte": 7850, "Contacto chapado oro": 8900, "PTFE": 2200, "Panel CIGS flexible": 1600}


def mass_props(tris, rho):
    """masa, COM e inercia (sobre el COM) de una malla cerrada por suma de tetraedros con el origen"""
    V = 0.0
    C = Vector((0, 0, 0))
    # integrales de segundo orden
    xx = yy = zz = xy = yz = zx = 0.0
    for a, b, c in tris:
        v = a.dot(b.cross(c)) / 6.0
        V += v
        C += v * (a + b + c) / 4.0
        for (i, j, acc) in ((0, 0, 'xx'), (1, 1, 'yy'), (2, 2, 'zz'), (0, 1, 'xy'), (1, 2, 'yz'), (2, 0, 'zx')):
            s = (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + (a[i] + b[i] + c[i]) * (a[j] + b[j] + c[j])) / 20.0
            if acc == 'xx': xx += v * s
            elif acc == 'yy': yy += v * s
            elif acc == 'zz': zz += v * s
            elif acc == 'xy': xy += v * s
            elif acc == 'yz': yz += v * s
            else: zx += v * s
    if abs(V) < 1e-12:
        return 0.0, Vector((0, 0, 0)), [0] * 6
    sgn = 1 if V > 0 else -1           # orientacion de normales
    V, xx, yy, zz, xy, yz, zx = [sgn * q for q in (V, xx, yy, zz, xy, yz, zx)]
    C = C * sgn / V
    m = rho * V
    # tensor sobre el origen -> sobre el COM
    Ixx = rho * (yy + zz) - m * (C.y ** 2 + C.z ** 2)
    Iyy = rho * (xx + zz) - m * (C.x ** 2 + C.z ** 2)
    Izz = rho * (xx + yy) - m * (C.x ** 2 + C.y ** 2)
    Ixy = -rho * xy + m * C.x * C.y
    Iyz = -rho * yz + m * C.y * C.z
    Ixz = -rho * zx + m * C.x * C.z
    return m, C, [Ixx, Ixy, Ixz, Iyy, Iyz, Izz]


def write_stl(path, tris):
    import struct
    with open(path, "wb") as f:
        f.write(b"LRM ROBOTICS V1".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            n = (b - a).cross(c - a)
            n = n.normalized() if n.length > 0 else n
            f.write(struct.pack("<12fH", *n, *a, *b, *c, 0))


link_data = {}
dg = bpy.context.evaluated_depsgraph_get()
for L in links:
    inv = L.matrix_world.inverted()
    all_tris = []
    m_tot, c_acc, parts_in = 0.0, Vector((0, 0, 0)), []
    comps = []
    for p in parts:
        if p.get("kin_link") != L.name:
            continue
        me = p.evaluated_get(dg).to_mesh()
        bm = bmesh.new()
        bm.from_mesh(me)
        bmesh.ops.triangulate(bm, faces=bm.faces)
        M = inv @ p.matrix_world
        tris = [tuple(M @ v.co for v in f.verts) for f in bm.faces]
        bm.free()
        p.evaluated_get(dg).to_mesh_clear()
        rho = DENS.get(p["material"], 1200)
        m, C, I = mass_props(tris, rho)
        comps.append((m, C, I))
        all_tris += tris
        parts_in.append(p.name)
    if comps:
        m_tot = sum(c[0] for c in comps)
        com = sum((c[0] * c[1] for c in comps), Vector((0, 0, 0))) / max(m_tot, 1e-9)
        # teorema de ejes paralelos para combinar
        I = [0.0] * 6
        for m, C, Ic in comps:
            d = C - com
            I[0] += Ic[0] + m * (d.y ** 2 + d.z ** 2)
            I[3] += Ic[3] + m * (d.x ** 2 + d.z ** 2)
            I[5] += Ic[5] + m * (d.x ** 2 + d.y ** 2)
            I[1] += Ic[1] - m * d.x * d.y
            I[4] += Ic[4] - m * d.y * d.z
            I[2] += Ic[2] - m * d.x * d.z
        mesh_file = f"{L.name}.stl"
        write_stl(os.path.join(MESH_DIR, mesh_file), all_tris)
    else:
        m_tot, com, I, mesh_file = 0.001, Vector((0, 0, 0)), [1e-8, 0, 0, 1e-8, 0, 1e-8], None
    # inercia minima fisicamente valida para el simulador
    I[0], I[3], I[5] = max(I[0], 1e-8), max(I[3], 1e-8), max(I[5], 1e-8)
    link_data[L.name] = dict(mass=max(m_tot, 0.001), com=com, I=I, mesh=mesh_file, parts=parts_in)


def rpy(R):
    e = R.to_euler('XYZ')
    return f"{e.x:.6f} {e.y:.6f} {e.z:.6f}"


def xyz(v):
    return f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}"


# esfuerzo (N*m) y velocidad (rad/s) por tipo de actuador
def actuation(name):
    if name == "elbow_joint":
        return 2.0, 1.5        # servo ~20 kg*cm a traves de corona z40/pinon z24 -> ~3.3 N*m pico; continuo 2
    if name == "wrist_joint":
        return 0.6, 1.0        # pasiva con friccion (bloqueo por anillo)
    if name.startswith("thumb_cmc"):
        return 0.15, 3.0       # micro servo de oposicion
    return 0.25, 4.0           # flexion por tendon (MG90S ~2 kg*cm sobre polea)


lines = ['<?xml version="1.0"?>',
         '<!-- LRM ROBOTICS - Protesis transhumeral V1. Generado por scripts/lrm_urdf.py -->',
         '<!-- Unidades SI (m, kg, rad). Z arriba, brazo derecho hacia +X, palma -Z. Base fija = socket. -->',
         '<robot name="lrm_protesis_v1">',
         '  <material name="rPET"><color rgba="0.10 0.55 0.50 1"/></material>']
for name, L in sorted(((e.name, e) for e in links), key=lambda t: len(t[1].children_recursive), reverse=True):
    d = link_data[name]
    I = d["I"]
    lines.append(f'  <link name="{name}">')
    lines.append(f'    <!-- piezas: {escape(", ".join(d["parts"]))} -->')
    lines.append('    <inertial>')
    lines.append(f'      <origin xyz="{xyz(d["com"])}" rpy="0 0 0"/>')
    lines.append(f'      <mass value="{d["mass"]:.5f}"/>')
    lines.append(f'      <inertia ixx="{I[0]:.4e}" ixy="{I[1]:.4e}" ixz="{I[2]:.4e}" iyy="{I[3]:.4e}" '
                 f'iyz="{I[4]:.4e}" izz="{I[5]:.4e}"/>')
    lines.append('    </inertial>')
    if d["mesh"]:
        for tag in ("visual", "collision"):
            lines.append(f'    <{tag}>')
            lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            lines.append(f'      <geometry><mesh filename="meshes/{d["mesh"]}" scale="1 1 1"/></geometry>')
            if tag == "visual":
                lines.append('      <material name="rPET"/>')
            lines.append(f'    </{tag}>')
    lines.append('  </link>')

joints_out = []
for L in links:
    if L.parent is None:
        continue
    T = L.parent.matrix_world.inverted() @ L.matrix_world
    axis = Vector(L["axis_local"]).normalized()
    lo, hi = [math.radians(a) for a in L["limit_deg"]]
    eff, vel = actuation(L.name)
    lines.append(f'  <joint name="{L.name}" type="revolute">')
    lines.append(f'    <!-- {escape(L["descripcion"])} | reposo Blender: {L["rest_deg"]} deg -->')
    lines.append(f'    <parent link="{L.parent.name}"/>')
    lines.append(f'    <child link="{L.name}"/>')
    lines.append(f'    <origin xyz="{xyz(T.to_translation())}" rpy="{rpy(T.to_3x3())}"/>')
    lines.append(f'    <axis xyz="{xyz(axis)}"/>')
    lines.append(f'    <limit lower="{lo:.5f}" upper="{hi:.5f}" effort="{eff}" velocity="{vel}"/>')
    lines.append('    <dynamics damping="0.02" friction="0.01"/>')
    lines.append('  </joint>')
    joints_out.append(L.name)
lines.append('</robot>')

urdf_path = os.path.join(OUT, "lrm_protesis_v1.urdf")
with open(urdf_path, "w", encoding="utf-8") as f:
    f.write("\n".join(lines) + "\n")

total = sum(d["mass"] for d in link_data.values())
print(f"[URDF] {urdf_path}")
print(f"[URDF] eslabones={len(link_data)} juntas={len(joints_out)} masa_total={total:.3f} kg (sin electronica)")
for n, d in sorted(link_data.items(), key=lambda t: -t[1]["mass"]):
    print(f"   {n:32s} {d['mass'] * 1000:8.1f} g  piezas={len(d['parts'])}")
