# -*- coding: utf-8 -*-
"""
LRM ROBOTICS - Protesis bionica transhumeral V1
Generador parametrico para Blender 5.2 (solo estructura, hueca, modular).

Ejecutar:
  blender -b --python scripts/lrm_protesis_v1.py

Convenciones
  * 1 unidad Blender = 1 m, la escena muestra milimetros. Exportes STL en mm.
  * Z arriba. Brazo DERECHO extendido hacia +X, palma hacia -Z (pronada),
    pulgar hacia +Y (medial). Origen = cara proximal del socket.
  * Cada pieza fisica es un objeto independiente; las juntas son Empties con
    el pivote exactamente en el eje de giro, limites y eje como propiedades.
"""
import bpy, bmesh, math, os, json
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

PROJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BLEND_OUT = os.path.join(PROJ, "LRM_Protesis_V1.blend")
KIN_OUT = os.path.join(PROJ, "exports", "LRM_V1_cinematica.json")

N_RING = 72          # resolucion de secciones
P_EXP = 2.5          # exponente superelipse (secciones redondeadas, no circulares)
WALL = 0.0025        # pared de carcasas rPET (2.5 mm)

# ---------------------------------------------------------------- escena base
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
us = scene.unit_settings
us.system, us.length_unit, us.scale_length = 'METRIC', 'MILLIMETERS', 1.0


def get_coll(name, parent=None):
    c = bpy.data.collections.get(name)
    if c is None:
        c = bpy.data.collections.new(name)
        (parent or scene.collection).children.link(c)
    return c


TMP = get_coll("_tmp")
PROT = get_coll("LRM_PROTESIS_V1")
KIN = get_coll("00_CINEMATICA_juntas", PROT)
C_ARM = get_coll("01_BRAZO_transhumeral", PROT)
C_ELB = get_coll("02_CODO", PROT)
C_FOR = get_coll("03_ANTEBRAZO", PROT)
C_WRI = get_coll("04_MUNECA", PROT)
C_HAND = get_coll("05_MANO_palma", PROT)
C_FING = get_coll("06_DEDOS", PROT)
C_THUMB = get_coll("07_PULGAR", PROT)
C_TEND = get_coll("08_TENDONES_virtuales", PROT)


# ---------------------------------------------------------------- utilidades malla
def new_obj(name, me, coll=TMP):
    o = bpy.data.objects.new(name, me)
    coll.objects.link(o)
    return o


def bm_to_obj(bm, name="tmp"):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    return new_obj(name, me)


def delete(o):
    me = o.data if o.type == 'MESH' else None
    bpy.data.objects.remove(o, do_unlink=True)
    if me is not None and me.users == 0:
        bpy.data.meshes.remove(me)


def dup(o, name="dup"):
    return new_obj(name, o.data.copy())


def apply_mods(o):
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg))
    old = o.data
    o.modifiers.clear()
    o.data = me
    if old.users == 0:
        bpy.data.meshes.remove(old)
    return o


def boolean(o, cutter, op='DIFFERENCE', keep=False):
    m = o.modifiers.new("b", 'BOOLEAN')
    m.operation, m.solver, m.object = op, 'EXACT', cutter
    apply_mods(o)
    if not keep:
        delete(cutter)
    return o


def cut(o, *cutters):
    for c in cutters:
        boolean(o, c, 'DIFFERENCE')
    return o


def union(o, *others):
    for c in others:
        boolean(o, c, 'UNION')
    return o


def intersect(o, cutter, keep=False):
    return boolean(o, cutter, 'INTERSECT', keep)


def solidify(o, t, offset=-1.0):
    m = o.modifiers.new("s", 'SOLIDIFY')
    m.thickness, m.offset = t, offset
    m.use_even_offset, m.use_quality_normals = True, True
    return apply_mods(o)


def bevel(o, w, seg=3, angle=35.0):
    m = o.modifiers.new("bv", 'BEVEL')
    m.width, m.segments = w, seg
    m.limit_method, m.angle_limit = 'ANGLE', math.radians(angle)
    m.use_clamp_overlap = True
    return apply_mods(o)


def transform(o, M):
    o.data.transform(M)
    return o


def join(objs, name="join"):
    bm = bmesh.new()
    for o in objs:
        bm.from_mesh(o.data)
        delete(o)
    return bm_to_obj(bm, name)


# ---------------------------------------------------------------- primitivas
def ring(x, w, h, zc=0.0, yc=0.0, n=N_RING, p=P_EXP):
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        pts.append(Vector((x,
                           yc + (w / 2) * math.copysign(abs(c) ** (2 / p), c),
                           zc + (h / 2) * math.copysign(abs(s) ** (2 / p), s))))
    return pts


def interp_keys(keys, sub=6):
    out, n = [], len(keys)
    for i in range(n - 1):
        p0, p1, p2, p3 = keys[max(i - 1, 0)], keys[i], keys[i + 1], keys[min(i + 2, n - 1)]
        for s in range(sub):
            t = s / sub
            out.append(tuple(0.5 * (2 * b + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t * t
                                    + (-a + 3 * b - 3 * c + d) * t ** 3)
                             for a, b, c, d in zip(p0, p1, p2, p3)))
    out.append(tuple(keys[-1]))
    return out


def sec_at(keys, x):
    """seccion (w,h,zc) interpolada en x (para calcular superficies)"""
    secs = interp_keys(keys, 12)
    for a, b in zip(secs[:-1], secs[1:]):
        if a[0] <= x <= b[0]:
            t = (x - a[0]) / (b[0] - a[0])
            return tuple(a[i] + (b[i] - a[i]) * t for i in range(1, 4))
    s = secs[0] if x < secs[0][0] else secs[-1]
    return s[1], s[2], s[3]


def offset_keys(keys, d):
    return [(k[0], k[1] - 2 * d, k[2] - 2 * d, k[3]) for k in keys]


def loft(keys, cap0=False, cap1=False, sub=6, name="loft"):
    secs = interp_keys(keys, sub)
    bm = bmesh.new()
    rings = []
    for s in secs:
        rings.append([bm.verts.new(v) for v in ring(s[0], s[1], s[2], s[3] if len(s) > 3 else 0.0)])
    n = N_RING
    for a, b in zip(rings[:-1], rings[1:]):
        for j in range(n):
            bm.faces.new((a[j], a[(j + 1) % n], b[(j + 1) % n], b[j]))
    if cap0:
        bm.faces.new(list(reversed(rings[0])))
    if cap1:
        bm.faces.new(rings[-1])
    bm.normal_update()
    return bm_to_obj(bm, name)


def shell(keys, t=WALL, cap0=False, cap1=False, name="shell"):
    return solidify(loft(keys, cap0, cap1, name=name), t, -1.0)


def solid(keys, name="solid"):
    return loft(keys, True, True, name=name)


def axis_matrix(center, axis):
    axis = Vector(axis).normalized()
    return Matrix.Translation(Vector(center)) @ Vector((0, 0, 1)).rotation_difference(axis).to_matrix().to_4x4()


def lathe(profile, center, axis=(1, 0, 0), segs=56, name="lathe"):
    """perfil cerrado [(r, a)] con r>0 (anillos, bujes, tubos)"""
    M = axis_matrix(center, axis)
    bm = bmesh.new()
    grid = []
    for k in range(segs):
        ang = 2 * math.pi * k / segs
        c, s = math.cos(ang), math.sin(ang)
        grid.append([bm.verts.new(M @ Vector((r * c, r * s, a))) for (r, a) in profile])
    m = len(profile)
    for k in range(segs):
        A, B = grid[k], grid[(k + 1) % segs]
        for i in range(m):
            i2 = (i + 1) % m
            bm.faces.new((A[i], A[i2], B[i2], B[i]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm_to_obj(bm, name)


def tube(r_out, r_in, a0, a1, center, axis=(1, 0, 0), segs=48, name="tube"):
    return lathe([(r_in, a0), (r_out, a0), (r_out, a1), (r_in, a1)], center, axis, segs, name)


def cyl(r, depth, center, axis=(0, 0, 1), segs=48, name="cyl"):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segs, radius1=r, radius2=r,
                          depth=depth, matrix=axis_matrix(center, axis))
    return bm_to_obj(bm, name)


def box(size, center, rot=None, name="box"):
    bm = bmesh.new()
    M = Matrix.Translation(Vector(center))
    if rot is not None:
        M = M @ rot.to_4x4()
    M = M @ Matrix.Diagonal(Vector((size[0], size[1], size[2], 1.0)))
    bmesh.ops.create_cube(bm, size=1.0, matrix=M)
    return bm_to_obj(bm, name)


def box_mm(x0, x1, y0, y1, z0, z1, name="box"):
    return box((x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), name=name)


def rbox(x0, x1, y0, y1, z0, z1, r, seg=4, name="rbox"):
    o = box_mm(x0, x1, y0, y1, z0, z1, name)
    m = o.modifiers.new("bv", 'BEVEL')
    m.width, m.segments, m.limit_method, m.use_clamp_overlap = r, seg, 'NONE', True
    return apply_mods(o)


def sphere(r, center, name="sph"):
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=40, v_segments=20, radius=r,
                              matrix=Matrix.Translation(Vector(center)))
    return bm_to_obj(bm, name)


def prism(poly, depth, plane='XZ', offset=0.0, name="prism"):
    """poligono 2D extruido. XZ: (u,v)=(x,z) extruido en y. XY: (x,y) extruido en z."""
    def P(u, v, w):
        if plane == 'XZ':
            return Vector((u, w, v))
        if plane == 'XY':
            return Vector((u, v, w))
        return Vector((w, u, v))  # YZ
    bm = bmesh.new()
    a = [bm.verts.new(P(u, v, offset - depth / 2)) for u, v in poly]
    b = [bm.verts.new(P(u, v, offset + depth / 2)) for u, v in poly]
    n = len(poly)
    caps = [bm.faces.new(a), bm.faces.new(list(reversed(b)))]
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((a[i], a[j], b[j], b[i]))
    bmesh.ops.triangulate(bm, faces=caps, quad_method='BEAUTY', ngon_method='EAR_CLIP')
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm_to_obj(bm, name)


def arc_pts(cx, cz, r, a0, a1, n=24):
    return [(cx + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             cz + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]


def stadium_xz(x0, cx, cz, half_h, n=24):
    """perfil: rectangulo desde x0 hasta el pivote + semicirculo de radio half_h alrededor del pivote
    (el semicirculo mira en sentido opuesto a x0)"""
    if x0 < cx:
        return [(x0, cz - half_h)] + arc_pts(cx, cz, half_h, -90, 90, n) + [(x0, cz + half_h)]
    return [(x0, cz + half_h)] + arc_pts(cx, cz, half_h, 90, 270, n) + [(x0, cz - half_h)]


def sector_poly(cx, cz, r0, r1, a0, a1, n=32):
    return arc_pts(cx, cz, r1, a0, a1, n) + list(reversed(arc_pts(cx, cz, r0, a0, a1, n)))


def sector_prism(cx, cz, r0, r1, a0, a1, depth, yc, n=40, name="sector"):
    """sector de anillo en XZ extruido en Y, construido con quads (sin n-gonos concavos)"""
    bm = bmesh.new()
    rows = []
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        c, s = math.cos(a), math.sin(a)
        rows.append([bm.verts.new((cx + r * c, yc + dy, cz + r * s))
                     for r, dy in ((r0, -depth / 2), (r1, -depth / 2), (r1, depth / 2), (r0, depth / 2))])
    for a_, b_ in zip(rows[:-1], rows[1:]):
        for j in range(4):
            bm.faces.new((a_[j], a_[(j + 1) % 4], b_[(j + 1) % 4], b_[j]))
    bm.faces.new(rows[0])
    bm.faces.new(list(reversed(rows[-1])))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm_to_obj(bm, name)


def rounded_trap(x0, x1, hw0, hw1, r, off=0.0, n=8):
    """trapecio (en XY) de x0..x1, semiancho hw0->hw1, esquinas redondeadas, desplazado 'off' hacia afuera"""
    x0, x1, hw0, hw1 = x0 - off, x1 + off, hw0 + off, hw1 + off
    corners = [(x0 + r, -hw0 + r, 180, 270), (x1 - r, -hw1 + r, 270, 360),
               (x1 - r, hw1 - r, 0, 90), (x0 + r, hw0 - r, 90, 180)]
    pts = []
    for cx, cy, a0, a1 in corners:
        for i in range(n + 1):
            a = math.radians(a0 + (a1 - a0) * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def smooth_path(pts, sub=8):
    return [Vector(p) for p in interp_keys([tuple(p) for p in pts], sub)]


def curve_mesh(points, r, name="crv", res=3):
    cu = bpy.data.curves.new(name, 'CURVE')
    cu.dimensions, cu.bevel_depth, cu.bevel_resolution, cu.use_fill_caps = '3D', r, res, True
    sp = cu.splines.new('POLY')
    sp.points.add(len(points) - 1)
    for p, co in zip(sp.points, points):
        p.co = (co[0], co[1], co[2], 1.0)
    o = bpy.data.objects.new(name, cu)
    TMP.objects.link(o)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(o.evaluated_get(dg))
    bpy.data.objects.remove(o)
    bpy.data.curves.remove(cu)
    return new_obj(name, me)


def helix(center, axis, R, wire, length, turns, name="helix"):
    M = axis_matrix(center, axis)
    n = int(turns * 24)
    pts = [M @ Vector((R * math.cos(2 * math.pi * turns * i / n), R * math.sin(2 * math.pi * turns * i / n),
                       -length / 2 + length * i / n)) for i in range(n + 1)]
    return curve_mesh(pts, wire, name, 2)


def sweep_rect(points, width, thick, hint=(0, 0, 1), name="strap"):
    bm = bmesh.new()
    pts = [Vector(p) for p in points]
    secs = []
    for i, p in enumerate(pts):
        T = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        S = T.cross(Vector(hint)).normalized()
        N = S.cross(T).normalized()
        secs.append([bm.verts.new(p + S * sx * width / 2 + N * nz * thick / 2)
                     for sx, nz in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    for a, b in zip(secs[:-1], secs[1:]):
        for j in range(4):
            bm.faces.new((a[j], a[(j + 1) % 4], b[(j + 1) % 4], b[j]))
    bm.faces.new(secs[0])
    bm.faces.new(list(reversed(secs[-1])))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm_to_obj(bm, name)


def gear_poly(cx, cz, r_pitch, teeth):
    m = 2 * r_pitch / teeth
    r_root, r_tip = r_pitch - 1.25 * m, r_pitch + m
    pa = 2 * math.pi / teeth
    pts = []
    for k in range(teeth):
        for f, r in ((0.0, r_root), (0.18, r_root), (0.33, r_tip), (0.57, r_tip), (0.72, r_root)):
            a = (k + f) * pa
            pts.append((cx + r * math.cos(a), cz + r * math.sin(a)))
    return pts


# ---------------------------------------------------------------- materiales
def principled(name, color, rough=0.5, metal=0.0, trans=0.0, ior=1.5, sss=0.0, coat=0.0, bump=0.0, bump_scale=900.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    b = nt.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Transmission Weight"].default_value = trans
    b.inputs["IOR"].default_value = ior
    b.inputs["Subsurface Weight"].default_value = sss
    b.inputs["Subsurface Radius"].default_value = (color[0] * 3 + .05, color[1] * 3 + .05, color[2] * 3 + .05)
    b.inputs["Subsurface Scale"].default_value = 0.004
    b.inputs["Coat Weight"].default_value = coat
    if bump > 0:
        tx = nt.nodes.new("ShaderNodeTexNoise")
        tx.inputs["Scale"].default_value = bump_scale
        tx.inputs["Detail"].default_value = 6
        bp = nt.nodes.new("ShaderNodeBump")
        bp.inputs["Strength"].default_value = bump
        bp.inputs["Distance"].default_value = 0.0002
        nt.links.new(tx.outputs["Fac"], bp.inputs["Height"])
        nt.links.new(bp.outputs["Normal"], b.inputs["Normal"])
    m.diffuse_color = (*color, 1)
    return m


MAT = {
    # rPET reciclado translucido (carcasas cosmeticas / estructurales exteriores)
    "rPET": principled("rPET_reciclado_translucido", (0.030, 0.30, 0.27), rough=0.30, trans=0.62, ior=1.57,
                       coat=0.25, bump=0.05),
    # rPET reforzado (EBA-GMA) opaco para piezas internas portantes (informe sec. 6.1)
    "rPET_int": principled("rPET_reforzado_interno", (0.018, 0.11, 0.11), rough=0.5, bump=0.08),
    "TPU": principled("TPU_negro_mate", (0.016, 0.017, 0.019), rough=0.72, sss=0.05, bump=0.35, bump_scale=1400),
    "silicona": principled("Silicona_medica_ISO10993", (0.05, 0.055, 0.06), rough=0.45, sss=0.25),
    "joint": principled("PET_negro_articulaciones", (0.025, 0.027, 0.03), rough=0.42, coat=0.2),
    "acero": principled("Acero_inox_satinado", (0.50, 0.51, 0.53), rough=0.42, metal=1.0),
    "resorte": principled("Acero_resorte", (0.32, 0.33, 0.35), rough=0.35, metal=1.0),
    "oro": principled("Contactos_oro", (1.0, 0.74, 0.32), rough=0.28, metal=1.0),
    "PTFE": principled("PTFE_guia_tendon", (0.86, 0.88, 0.90), rough=0.5, trans=0.25),
    "nylon": principled("Nylon_tendon_virtual", (0.95, 0.45, 0.05), rough=0.4),
}


def solar_material():
    m = bpy.data.materials.new("Panel_solar_flexible_CIGS")
    m.use_nodes = True
    nt = m.node_tree
    nd, lk = nt.nodes, nt.links
    b = nd["Principled BSDF"]
    tc = nd.new("ShaderNodeTexCoord")
    sep = nd.new("ShaderNodeSeparateXYZ")
    lk.new(tc.outputs["Generated"], sep.inputs[0])
    # mascara marco (borde 1.6 mm aprox)
    def band(inp, lo, hi):
        g1 = nd.new("ShaderNodeMath"); g1.operation = 'GREATER_THAN'; g1.inputs[1].default_value = lo
        g2 = nd.new("ShaderNodeMath"); g2.operation = 'LESS_THAN'; g2.inputs[1].default_value = hi
        lk.new(inp, g1.inputs[0]); lk.new(inp, g2.inputs[0])
        mu = nd.new("ShaderNodeMath"); mu.operation = 'MULTIPLY'
        lk.new(g1.outputs[0], mu.inputs[0]); lk.new(g2.outputs[0], mu.inputs[1])
        return mu.outputs[0]
    mx = band(sep.outputs["X"], 0.012, 0.988)
    my = band(sep.outputs["Y"], 0.03, 0.97)
    sepn = nd.new("ShaderNodeSeparateXYZ")
    lk.new(tc.outputs["Normal"], sepn.inputs[0])
    mz = band(sepn.outputs["Z"], 0.25, 2.0)   # solo cara superior (normal hacia arriba)
    cell = nd.new("ShaderNodeMath"); cell.operation = 'MULTIPLY'
    lk.new(mx, cell.inputs[0]); lk.new(my, cell.inputs[1])
    cell2 = nd.new("ShaderNodeMath"); cell2.operation = 'MULTIPLY'
    lk.new(cell.outputs[0], cell2.inputs[0]); lk.new(mz, cell2.inputs[1])
    # rejilla de celdas
    mp = nd.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (24.0, 6.0, 1.0)
    lk.new(tc.outputs["Generated"], mp.inputs["Vector"])
    br = nd.new("ShaderNodeTexBrick")
    br.offset = 0.0
    br.inputs["Color1"].default_value = (0.010, 0.020, 0.075, 1)
    br.inputs["Color2"].default_value = (0.014, 0.028, 0.095, 1)
    br.inputs["Mortar"].default_value = (0.24, 0.26, 0.30, 1)
    br.inputs["Scale"].default_value = 1.0
    br.inputs["Mortar Size"].default_value = 0.018
    br.inputs["Brick Width"].default_value = 1.0
    br.inputs["Row Height"].default_value = 1.0
    lk.new(mp.outputs[0], br.inputs["Vector"])
    mix = nd.new("ShaderNodeMix"); mix.data_type = 'RGBA'
    mix.inputs[6].default_value = (0.015, 0.015, 0.017, 1)  # marco TPU
    lk.new(cell2.outputs[0], mix.inputs[0])
    lk.new(br.outputs["Color"], mix.inputs[7])
    lk.new(mix.outputs[2], b.inputs["Base Color"])
    rmix = nd.new("ShaderNodeMapRange")
    rmix.inputs["To Min"].default_value = 0.7
    rmix.inputs["To Max"].default_value = 0.30
    lk.new(cell2.outputs[0], rmix.inputs["Value"])
    lk.new(rmix.outputs[0], b.inputs["Roughness"])
    # encapsulado mate antirreflejo (como los paneles CIGS flexibles reales): sin barniz brillante
    b.inputs["Specular IOR Level"].default_value = 0.2
    b.inputs["Coat Weight"].default_value = 0.08
    b.inputs["Coat Roughness"].default_value = 0.35
    m.diffuse_color = (0.03, 0.05, 0.14, 1)
    return m


MAT["solar"] = solar_material()


def resin_volume(m, color, density):
    """absorcion volumetrica: el color se intensifica con el espesor, como resina/PET translucido real"""
    nt = m.node_tree
    out = next(n for n in nt.nodes if n.bl_idname == "ShaderNodeOutputMaterial")
    va = nt.nodes.new("ShaderNodeVolumeAbsorption")
    va.inputs["Color"].default_value = (*color, 1)
    va.inputs["Density"].default_value = density
    nt.links.new(va.outputs[0], out.inputs["Volume"])


# rPET translucido lechoso: dispersion interna (subsurface) + algo de transmision para intuir lo de adentro
b = MAT["rPET"].node_tree.nodes["Principled BSDF"]
b.inputs["Base Color"].default_value = (0.016, 0.27, 0.235, 1)
b.inputs["Subsurface Weight"].default_value = 0.6
b.inputs["Subsurface Radius"].default_value = (0.08, 0.9, 0.7)
b.inputs["Subsurface Scale"].default_value = 0.008
b.inputs["Transmission Weight"].default_value = 0.3
b.inputs["Roughness"].default_value = 0.35
b.inputs["Coat Weight"].default_value = 0.3
MAT_TAG = {"rPET": ("rPET reciclado", "rigid"), "rPET_int": ("rPET reforzado EBA-GMA", "rigid"),
           "TPU": ("TPU", "flexible"), "silicona": ("Silicona medica platino ISO 10993", "flexible"),
           "joint": ("PET negro", "rigid"), "acero": ("Acero inox", "rigid"), "resorte": ("Acero resorte", "flexible"),
           "oro": ("Contacto chapado oro", "rigid"), "PTFE": ("PTFE", "rigid"), "solar": ("Panel CIGS flexible", "flexible")}

# ---------------------------------------------------------------- piezas y eslabones
PARTS = []
LINKS = {}


def make_link(name, loc, parent=None, rotm=None, jtype="fixed", axis=None, limits=None, rest=0.0, rot_axis='Y',
              desc=""):
    e = bpy.data.objects.new(name, None)
    e.empty_display_type = 'ARROWS'
    e.empty_display_size = 0.012
    KIN.objects.link(e)
    if rotm is not None:
        e.matrix_world = Matrix.Translation(Vector(loc)) @ rotm.to_4x4()
    else:
        e.location = loc
    bpy.context.view_layer.update()
    if parent is not None:
        # la orientacion de reposo va en parent_inverse -> rotacion local = 0 en reposo (angulo de junta puro)
        mw = e.matrix_world.copy()
        e.parent = parent
        e.matrix_parent_inverse = parent.matrix_world.inverted() @ mw
        e.matrix_basis = Matrix.Identity(4)
    bpy.context.view_layer.update()
    e["joint_type"] = jtype
    e["descripcion"] = desc
    if axis is not None:
        e["axis_local"] = list(axis)
        e["rot_channel"] = rot_axis
        e["limit_deg"] = list(limits)
        e["rest_deg"] = rest
        c = e.constraints.new('LIMIT_ROTATION')
        c.owner_space = 'LOCAL'
        c.use_limit_x = c.use_limit_y = c.use_limit_z = True
        lo, hi = limits
        # el signo se resuelve con el vector 'axis' (+1/-1 sobre el canal)
        sgn = sum(axis)
        a0, a1 = sorted((math.radians(lo * sgn), math.radians(hi * sgn)))
        for ch in 'xyz':
            setattr(c, 'min_' + ch, 0.0)
            setattr(c, 'max_' + ch, 0.0)
        setattr(c, 'min_' + rot_axis.lower(), a0)
        setattr(c, 'max_' + rot_axis.lower(), a1)
    LINKS[name] = e
    return e


def part(o, name, coll, mat, link, explode=(0, 0, 0), smooth=True, sharp=35.0, extra_mats=(), desc=""):
    for c in list(o.users_collection):
        c.objects.unlink(o)
    coll.objects.link(o)
    o.name = name
    o.data.name = name
    me = o.data
    vs = [v.co for v in me.vertices]
    if not vs:
        raise RuntimeError(f"[LRM] pieza vacia tras booleanas: {name}")
    mn = Vector((min(v.x for v in vs), min(v.y for v in vs), min(v.z for v in vs)))
    mx = Vector((max(v.x for v in vs), max(v.y for v in vs), max(v.z for v in vs)))
    ctr = (mn + mx) / 2
    me.transform(Matrix.Translation(-ctr))
    o.location = ctr
    me.materials.clear()     # las booleanas dejan slots vacios heredados de los cortadores
    me.materials.append(MAT[mat])
    for em in extra_mats:
        me.materials.append(MAT[em])
    if not extra_mats:
        me.polygons.foreach_set("material_index", [0] * len(me.polygons))
    if smooth:
        me.shade_smooth()
        me.set_sharp_from_angle(angle=math.radians(sharp))
    o.parent = link
    o.matrix_parent_inverse = link.matrix_world.inverted()
    mtxt, phys = MAT_TAG[mat]
    o["material"] = mtxt
    o["physics"] = phys
    o["kin_link"] = link.name
    o["descripcion"] = desc
    o["size_mm"] = [round((mx - mn)[i] * 1000, 2) for i in range(3)]
    PARTS.append(dict(obj=o, explode=Vector(explode), lift=None))
    return o


# =============================================================================
#  GEOMETRIA DE REFERENCIA (m)  -  usuario joven p50, brazo derecho ~70 cm
# =============================================================================
SOCKET_K = [(0.000, 0.118, 0.110, 0.0), (0.030, 0.110, 0.104, 0.0), (0.070, 0.100, 0.094, 0.0),
            (0.110, 0.092, 0.086, 0.0), (0.135, 0.088, 0.082, 0.0), (0.146, 0.086, 0.080, 0.0)]
ARMCOV_K = [(0.150, 0.084, 0.078, 0.0), (0.185, 0.080, 0.074, 0.0), (0.220, 0.076, 0.070, 0.0)]
ELBH_K = [(0.226, 0.076, 0.070, 0.0), (0.250, 0.075, 0.066, 0.0), (0.272, 0.072, 0.058, 0.0)]
FORE_K = [(0.342, 0.080, 0.074, 0.004), (0.370, 0.081, 0.072, 0.004), (0.410, 0.077, 0.066, 0.003),
          (0.450, 0.070, 0.059, 0.002), (0.490, 0.062, 0.052, 0.001), (0.512, 0.057, 0.047, 0.000)]
PALM_K = [(0.552, 0.052, 0.041, 0.0), (0.566, 0.064, 0.040, 0.0), (0.586, 0.073, 0.036, 0.001),
          (0.606, 0.076, 0.032, 0.001), (0.622, 0.076, 0.029, 0.001), (0.630, 0.074, 0.027, 0.001)]
ELBOW_P = Vector((0.300, 0.0, 0.016))   # pivote desplazado hacia anterior para permitir 135 deg
ELBOW_CLEAR_H = 168.0   # el lado humeral queda por detras de esta recta (deg desde +X, alrededor del pivote)
ELBOW_CLEAR_F = 30.0    # el lado del antebrazo queda por debajo de esta recta en reposo (30 + 135 < 168)
WRIST_X = 0.5345

# ---------------------------------------------------------------- eslabones
L_ROOT = make_link("socket_arm", (0, 0, 0), desc="Raiz: cara proximal del socket transhumeral")
L_ELB = make_link("elbow_joint", ELBOW_P, L_ROOT, jtype="revolute", axis=(0, -1, 0), limits=(0, 135),
                  desc="Codo: flexion 0-135 deg, eje medio-lateral")
L_WRI = make_link("wrist_joint", (WRIST_X, 0, 0), L_ELB, jtype="revolute", axis=(1, 0, 0), limits=(-90, 90),
                  rot_axis='X', desc="Muneca: prono-supinacion +-90 deg, eje largo del antebrazo")

# =============================================================================
#  01 BRAZO TRANSHUMERAL
# =============================================================================
A = Vector((-0.06, 0, 0))   # desplazamiento base de explosion del grupo brazo


def trim_plane_cutter(off=0.0):
    n = Vector((1.0, -0.18, 0.0)).normalized()
    p0 = Vector((0.020 + off, 0, 0))
    rot = Vector((1, 0, 0)).rotation_difference(n).to_matrix()
    return box((0.2, 0.4, 0.4), p0 - n * 0.1, rot, name="trim")


# socket (rPET, hueco, linea de recorte medial baja para la axila)
o = shell(SOCKET_K, 0.004, cap1=True, name="socket")
cut(o, trim_plane_cutter())
emg_x = 0.085
for sgn in (1, -1):
    cut(o, cyl(0.0082, 0.03, (emg_x, 0, sgn * 0.045), (0, 0, 1)))
cut(o, cyl(0.0042, 0.012, (0.146, -0.020, 0.012), (1, 0, 0)))   # paso de cable EMG hacia el canal
socket = part(o, "socket_transhumeral", C_ARM, "rPET", L_ROOT, A + Vector((-0.11, 0, 0)),
              desc="Encaje transhumeral impreso rPET, pared 4 mm")

# liner silicona medica (contacto con piel - requisito del informe)
LINER_K = [(-0.004,) + tuple(SOCKET_K[0][1:])] + SOCKET_K[1:-1] + [(0.1416,) + sec_at(SOCKET_K, 0.1416)]
LINER_K = offset_keys(LINER_K, 0.0041)
o = shell(LINER_K, 0.003, cap1=True, name="liner")
cut(o, trim_plane_cutter(-0.006))
for sgn in (1, -1):
    cut(o, cyl(0.0082, 0.03, (emg_x, 0, sgn * 0.045), (0, 0, 1)))
part(o, "liner_silicona_medica", C_ARM, "silicona", L_ROOT, A + Vector((-0.20, 0, 0)),
     desc="Liner silicona grado medico curada por platino (ISO 10993-5), 3 mm")

# alojamientos EMG (x2) - atraviesan socket y liner hasta la piel
for sgn, nm in ((1, "biceps"), (-1, "triceps")):
    w, h, zc = sec_at(SOCKET_K, emg_x)
    z_in = sgn * (h / 2 - 0.0041 - 0.003 - 0.0005)     # 0.5 mm hacia la piel
    z_out = sgn * (h / 2 + 0.0045)
    o = cyl(0.0080, abs(z_out - z_in), (emg_x, 0, (z_in + z_out) / 2), (0, 0, 1), name="emg")
    zc_cav = z_in + sgn * (abs(z_out - z_in) - 0.0015) / 2 - sgn * 0.0005
    cut(o, cyl(0.0045, abs(z_out - z_in) - 0.0015 + 0.001, (emg_x, 0, zc_cav), (0, 0, 1)))
    bevel(o, 0.0006, 2, 40)
    part(o, f"emg_electrode_housing_{nm}", C_ARM, "TPU", L_ROOT,
         A + Vector((-0.11, 0, sgn * 0.085)), desc="Alojamiento TPU para electrodo sEMG (MyoWare)")

# placa de montaje del arnes (inclinada siguiendo la superficie)
def top_z(keys, x):
    w, h, zc = sec_at(keys, x)
    return zc + h / 2


x0, x1 = 0.026, 0.050
z0, z1 = top_z(SOCKET_K, x0), top_z(SOCKET_K, x1)
ang = math.atan2(z0 - z1, x1 - x0)
pc = Vector(((x0 + x1) / 2, 0, (z0 + z1) / 2 + 0.0019))
rot = Matrix.Rotation(ang, 3, 'Y')
o = box((0.026, 0.024, 0.003), pc, rot, name="plate")
bevel(o, 0.0012, 3, 30)
cut(o, cyl(0.0022, 0.02, pc, rot @ Vector((0, 0, 1))))
part(o, "harness_mounting_plate", C_ARM, "rPET_int", L_ROOT, A + Vector((-0.11, 0, 0.10)),
     desc="Placa de anclaje del arnes en 8")

# arnes de suspension (correa TPU, sale de la placa y pasa sobre el hombro)
p_start = pc + rot @ Vector((0.006, 0, 0.0029))
strap_pts = smooth_path([p_start, p_start + Vector((-0.012, 0, 0.004)), (-0.03, 0.005, 0.085),
                         (-0.075, 0.025, 0.115), (-0.125, 0.06, 0.112), (-0.16, 0.10, 0.08),
                         (-0.17, 0.14, 0.035)], 6)
o = sweep_rect(strap_pts, 0.025, 0.0022, hint=(0, 0, 1))
part(o, "suspension_harness", C_ARM, "TPU", L_ROOT, A + Vector((-0.17, 0, 0.13)),
     desc="Arnes de suspension en 8 (TPU), tramo del lado protesico")

# tapa proximal del brazo (union socket - funda)
o = solid([(0.1462,) + tuple(sec_at(SOCKET_K, 0.146)[:2]) + (0.0,), (0.1500, 0.085, 0.079, 0.0)])
cut(o, cyl(0.0042, 0.02, (0.148, 0.024, 0.014), (1, 0, 0)))
cut(o, cyl(0.0042, 0.02, (0.148, -0.020, 0.012), (1, 0, 0)))
part(o, "arm_proximal_cap", C_ARM, "rPET_int", L_ROOT, A + Vector((-0.06, 0, 0)),
     desc="Mamparo proximal: fija tubo humeral al socket")

# funda cosmetica del brazo
o = shell([(0.1503, 0.084, 0.078, 0.0), (0.185, 0.080, 0.074, 0.0), (0.2197, 0.076, 0.070, 0.0)], WALL)
part(o, "arm_cosmetic_cover", C_ARM, "rPET", L_ROOT, A + Vector((0.0, 0, 0.12)),
     desc="Funda cosmetica hueca rPET")

# tapa distal del brazo
o = solid([(0.2200, 0.076, 0.070, 0.0), (0.2258, 0.0758, 0.0698, 0.0)])
cut(o, cyl(0.0112, 0.02, (0.223, -0.022, 0.010), (1, 0, 0)), cyl(0.0042, 0.02, (0.223, 0.024, 0.014), (1, 0, 0)))
cut(o, rbox(0.218, 0.228, -0.016, 0.016, -0.026, -0.010, 0.003))   # paso de cables de bateria
part(o, "arm_distal_cap", C_ARM, "rPET_int", L_ROOT, A + Vector((0.035, 0, 0)),
     desc="Mamparo distal del brazo")

# tubo estructural humeral
o = tube(0.0110, 0.0085, 0.1502, 0.2320, (0, -0.022, 0.010), (1, 0, 0))
part(o, "humeral_structural_tube", C_ARM, "rPET_int", L_ROOT, A + Vector((0.0, -0.075, 0)),
     desc="Tubo portante humeral 22x17 mm (rPET reforzado)")

# canal interno de cables
o = tube(0.0040, 0.0028, 0.1503, 0.2445, (0, 0.024, 0.014), (1, 0, 0))
part(o, "internal_cable_channel", C_ARM, "rPET_int", L_ROOT, A + Vector((0.0, 0.07, 0.0)),
     desc="Canal de cables EMG/potencia")

# alojamiento bateria 2x18650 (hueco)
o = rbox(0.1506, 0.2194, -0.0205, 0.0205, -0.0290, -0.0065, 0.003)
cut(o, rbox(0.1521, 0.2179, -0.019, 0.019, -0.0275, -0.0080, 0.0018))
cut(o, box_mm(0.214, 0.222, -0.014, 0.014, -0.024, -0.012))      # salida de cables
part(o, "battery_housing", C_ARM, "rPET_int", L_ROOT, A + Vector((0.0, 0, -0.10)),
     desc="Alojamiento pack 2x18650 Li-ion (66x37x19 mm utiles)")

# =============================================================================
#  02 CODO
# =============================================================================
E = Vector((0.0, 0, 0))
px, pz = ELBOW_P.x, ELBOW_P.z

def _clear_frame(angle_deg, side):
    a = math.radians(angle_deg)
    n = Vector((-math.sin(a), 0, math.cos(a)))
    if side == 'humeral':
        n = -n
    return n, Vector((0, 0, 1)).rotation_difference(n).to_matrix()


def clearance_cutter(angle_deg, side):
    """semiespacio anterior a la recta que pasa por el pivote del codo (solo del lado de cada segmento)"""
    n, rot = _clear_frame(angle_deg, side)
    c = box((0.4, 0.4, 0.4), ELBOW_P + n * 0.2004, rot, name="clear")
    lim = box_mm(-1, ELBOW_P.x, -1, 1, -1, 1) if side == 'humeral' else box_mm(ELBOW_P.x, 2, -1, 1, -1, 1)
    return intersect(c, lim)


def clearance_wall(angle_deg, side, keys, t=WALL):
    """pared que cierra la carcasa hueca a lo largo del chaflan"""
    n, rot = _clear_frame(angle_deg, side)
    s = box((0.4, 0.4, t), ELBOW_P + n * (0.0004 - t / 2), rot, name="cwall")
    intersect(s, solid(keys))
    return s


# mitad proximal: carcasa hueca + orejas del eje, con chaflan anterior de holgura (cerrado con pared)
o = shell(ELBH_K, WALL)
cheeks = []
for sgn in (1, -1):
    ch = prism(stadium_xz(0.262, px, pz, 0.019), 0.0085, 'XZ', sgn * 0.03125)
    cheeks.append(ch)
union(o, *cheeks)
cut(o, clearance_cutter(ELBOW_CLEAR_H, 'humeral'))
union(o, clearance_wall(ELBOW_CLEAR_H, 'humeral', ELBH_K))
cut(o, cyl(0.0041, 0.09, ELBOW_P, (0, 1, 0)))
part(o, "elbow_body_proximal", C_ELB, "rPET", L_ROOT, E + Vector((0.0, 0, 0.0)),
     desc="Mitad proximal del codo: carcasa + orejas portantes")

# soporte del servo del codo (servo 40x20 mm, eje paralelo al codo; pinon engrana a 200 deg del pivote)
SERVO_SHAFT = ELBOW_P + 0.036 * Vector((math.cos(math.radians(220)), 0, math.sin(math.radians(220))))
sx, sz = SERVO_SHAFT.x, SERVO_SHAFT.z
o = box_mm(0.2265, 0.2805, 0.0105, 0.0135, sz - 0.0150, sz + 0.0150)
cut(o, box_mm(sx - 0.0302, sx + 0.0104, 0.0, 0.03, sz - 0.0102, sz + 0.0102))
for xh in (0.2312, 0.2785):
    for zh in (sz - 0.005, sz + 0.005):
        cut(o, cyl(0.0012, 0.02, (xh, 0.012, zh), (0, 1, 0)))
part(o, "elbow_servo_mount", C_ELB, "rPET_int", L_ROOT, E + Vector((-0.02, 0.0, 0.085)),
     desc="Marco de montaje servo alto torque 40x20 mm (DS3218 o similar)")

# eje del codo (hueco para tornillos de friccion)
o = tube(0.0040, 0.0021, -0.0354, 0.0354, ELBOW_P, (0, 1, 0))
part(o, "elbow_rotation_axis_pin", C_ELB, "acero", L_ROOT, E + Vector((0.03, 0.16, 0)),
     desc="Pasador de acero 8 mm")

# tapas laterales (x2) + tornillos de friccion (x2)
for sgn, nm in ((1, "medial"), (-1, "lateral")):
    yc = sgn * 0.03695
    o = cyl(0.021, 0.0025, (px, yc, pz), (0, 1, 0), 64)
    bevel(o, 0.0007, 2, 40)
    cut(o, cyl(0.0022, 0.01, (px, yc, pz), (0, 1, 0)))
    part(o, f"elbow_lateral_cover_{nm}", C_ELB, "rPET", L_ROOT, E + Vector((0.03, sgn * 0.085, 0)),
         desc="Tapa lateral del codo")
    head = cyl(0.0035, 0.0035, (px, sgn * 0.04017, pz), (0, 1, 0), 32)
    bevel(head, 0.0004, 2, 40)
    hexs = cyl(0.0016, 0.004, (px, sgn * 0.0420, pz), (0, 1, 0), 6)
    cut(head, hexs)
    shank = cyl(0.0019, 0.0090, (px, sgn * 0.0338, pz), (0, 1, 0), 24)
    o = join([head, shank])
    part(o, f"elbow_friction_screw_{nm}", C_ELB, "acero", L_ROOT, E + Vector((0.03, sgn * 0.125, 0)),
         desc="Tornillo de friccion ajustable M4 (precarga de arandelas)")

# limitador de angulo 0-135 (sector fijo en la oreja lateral)
LUG_REST = -100.0
o = sector_prism(px, pz, 0.013, 0.022, LUG_REST + 7.5 + 135.0, 360 + LUG_REST - 7.5, 0.0073, -0.02315)
part(o, "elbow_angle_limiter", C_ELB, "rPET_int", L_ROOT, E + Vector((0.03, -0.06, 0)),
     desc="Tope mecanico: el talon del antebrazo recorre 0-135 deg")

# mitad distal (lengueta solidaria al antebrazo)
o = prism(stadium_xz(0.3380, px, pz, 0.018), 0.036, 'XZ', 0.0)
cut(o, clearance_cutter(15.0, 'forearm'))     # cuna anterior: a 135 deg no invade el lado humeral
lug = sector_prism(px, pz, 0.0135, 0.0215, LUG_REST - 7, LUG_REST + 7, 0.0076, -0.0217)
union(o, lug)
cut(o, cyl(0.0041, 0.06, ELBOW_P, (0, 1, 0)))
cut(o, cyl(0.0050, 0.05, (0.329, 0, pz), (1, 0, 0)), cyl(0.0045, 0.03, (0.313, 0, pz + 0.012), (0, 0, 1)))
part(o, "elbow_body_distal", C_ELB, "rPET_int", L_ELB, E + Vector((0.06, 0, 0)),
     desc="Mitad distal del codo (lengueta + talon de tope), paso de cables")

# engranaje de transmision (z=40, m=1.125) solidario al antebrazo
o = prism(gear_poly(px, pz, 0.0225, 40), 0.006, 'XZ', 0.0225)
cut(o, cyl(0.0041, 0.02, ELBOW_P, (0, 1, 0)))
for k in range(6):
    a = 2 * math.pi * k / 6
    cut(o, cyl(0.0040, 0.02, (px + 0.0132 * math.cos(a), 0.0225, pz + 0.0132 * math.sin(a)), (0, 1, 0), 32))
part(o, "elbow_transmission_gear", C_ELB, "rPET_int", L_ELB, E + Vector((0.06, 0.05, 0)), sharp=50,
     desc="Corona z40 m1.125 (pinon z24 en el servo, distancia entre ejes 36 mm)")

# =============================================================================
#  03 ANTEBRAZO
# =============================================================================
F = Vector((0.10, 0, 0))
CH_X0, CH_X1, CH_HW0, CH_HW1, CH_R = 0.350, 0.502, 0.030, 0.022, 0.006   # canal solar ~83 cm2


def channel_cutter(off=0.0, z0=None, z1=0.08):
    z0 = 0.012 if z0 is None else z0
    return prism(rounded_trap(CH_X0, CH_X1, CH_HW0, CH_HW1, CH_R, off), z1 - z0, 'XY', (z0 + z1) / 2, name="chan")


# tapa proximal del antebrazo (mamparo con ranura para la lengueta)
o = solid([(0.3362, 0.0796, 0.0736, 0.004), (0.3420, 0.0800, 0.0740, 0.004)])
cut(o, box_mm(0.3355, 0.3384, -0.0182, 0.0182, pz - 0.0182, pz + 0.0182))
cut(o, cyl(0.0050, 0.02, (0.339, 0, pz), (1, 0, 0)))
cut(o, clearance_cutter(ELBOW_CLEAR_F, 'forearm'))
part(o, "forearm_proximal_cap", C_FOR, "rPET_int", L_ELB, F + Vector((-0.025, 0, 0)),
     desc="Mamparo proximal del antebrazo, recibe la lengueta del codo")

# carcasa principal (se recorta: tapa lateral, ventana solar, tapa bateria, muesca)
fore = shell([(0.3422,) + FORE_K[0][1:]] + FORE_K[1:-1] + [(0.5118,) + FORE_K[-1][1:]], WALL, name="fore")
lat_cut = lambda off: rbox(0.372 - off, 0.470 + off, -0.08, -0.018 + off, -0.020 - off, 0.024 + off, 0.006)
# tapa lateral
o = dup(fore)
intersect(o, lat_cut(0.0))
cut(o, channel_cutter(0.0004))
part(o, "forearm_lateral_cosmetic_cover", C_FOR, "rPET", L_ELB, F + Vector((0, -0.10, 0)),
     desc="Tapa lateral removible: acceso a servos y electronica")
# tapa de bateria (lado ventral)
hz = -0.018   # plano superior del recorte ventral (relativo a zc~0.004)
def hatch_cutter(off):
    return prism(rounded_trap(0.356 - off, 0.408 + off, 0.0165 + off, 0.0165 + off, 0.005), 0.06 + off, 'XY',
                 hz - 0.03 + off / 2)
o = dup(fore)
intersect(o, hatch_cutter(0.0))
part(o, "lipo_compartment_lid", C_FOR, "rPET", L_ELB, F + Vector((0, 0, -0.13)),
     desc="Tapa abatible del compartimento LiPo/supercapacitor (sin herramientas)")
# cortes en la carcasa
cut(fore, lat_cut(0.0004), hatch_cutter(0.0004), channel_cutter(0.0), clearance_cutter(ELBOW_CLEAR_F, 'forearm'))
cut(fore, cyl(0.006, 0.05, (CH_X1 - 0.0015, 0, 0.045), (0, 0, 1)))   # muesca para levantar el panel
part(fore, "forearm_main_housing", C_FOR, "rPET", L_ELB, F, desc="Carcasa principal hueca del antebrazo")

# base / bandeja del panel solar (canal rebajado)
TRAY_OFF = WALL + 0.0001
o = shell(offset_keys(FORE_K, TRAY_OFF), 0.002, name="tray")
intersect(o, channel_cutter(0.004, z0=0.008))
cut(o, box_mm(0.3555, 0.3625, -0.0055, 0.0055, 0.0, 0.06))      # ranura del clip
cut(o, box_mm(0.3658, 0.3782, -0.0082, 0.0082, 0.0, 0.06))      # hueco del conector
part(o, "solar_panel_support_base", C_FOR, "rPET_int", L_ELB, F + Vector((0, 0, 0.10)),
     desc="Bandeja rebajada del canal solar (fondo del canal)")


def tray_top(x):
    w, h, zc = sec_at(FORE_K, x)
    return zc + h / 2 - TRAY_OFF


# conector de contacto (4 pines) del panel
tt = tray_top(0.378)
blk = rbox(0.3662, 0.3778, -0.0078, 0.0078, tt - 0.0040, tt - 0.0012, 0.0008)
pads = [box_mm(0.3680, 0.3760, yy - 0.0011, yy + 0.0011, tt - 0.0013, tt - 0.0008) for yy in (-0.0045, -0.0015, 0.0015, 0.0045)]
padj = join(pads)
for p in padj.data.polygons:
    p.material_index = 1
o = join([blk, padj])
part(o, "solar_contact_connector", C_FOR, "TPU", L_ELB, F + Vector((0, 0, 0.125)), extra_mats=("oro",),
     desc="Conector de contacto con 4 pads dorados (carga solar)")

# panel solar flexible (pieza independiente, encastre a ras)
o = shell(offset_keys(FORE_K, 0.0002), 0.0022, name="panel")
intersect(o, channel_cutter(-0.0004, z0=0.0))
tb = tray_top(0.359)
tab = box_mm(0.3560, 0.3620, -0.005, 0.005, tb - 0.0045, tb + 0.0007)
lip = box_mm(0.3535, 0.3620, -0.005, 0.005, tb - 0.0045, tb - 0.0027)
o = union(o, union(tab, lip))
solar = part(o, "solar_panel_dorsal_forearm", C_FOR, "solar", L_ELB, F + Vector((0, 0, 0.17)), sharp=60,
             desc="Panel solar flexible CIGS ~83 cm2 con marco/labio de encastre y clip inferior")
PARTS[-1]["lift"] = (Vector((0.0, 0.0, 0.055)), math.radians(-14))

# cubierta modulo de carga TP4056 (hueca, abierta abajo)
tp_top = tray_top(0.400) - 0.0021 - 0.0004
o = rbox(0.3820, 0.4000, -0.011, 0.011, 0.0240, tp_top, 0.002)
cut(o, box_mm(0.3835, 0.3985, -0.0095, 0.0095, 0.02, tp_top - 0.0015))
part(o, "solar_charge_module_cover_TP4056", C_FOR, "rPET_int", L_ELB, F + Vector((0, 0.06, 0.07)),
     desc="Cubierta del cargador TP4056 + diodo de bloqueo")

# compartimento Arduino/ESP32 (cubierta-estante conformada a la carcasa + paredes)
inner = solid(offset_keys(FORE_K, WALL + 0.0003))
deck = dup(inner); intersect(deck, box_mm(0.348, 0.408, -0.1, 0.1, 0.0020, 0.0040))
walls = box_mm(0.348, 0.408, -0.017, 0.017, 0.0038, 0.0205)
cut(walls, box_mm(0.3495, 0.4065, -0.0155, 0.0155, 0.0030, 0.03))
o = union(deck, walls)
part(o, "electronics_compartment_ESP32", C_FOR, "rPET_int", L_ELB, F + Vector((-0.01, 0.08, 0.05)),
     desc="Bahia ESP32/Arduino Nano 58x34 mm sobre estante portante")

# compartimento LiPo / supercapacitor (paredes hasta la carcasa, abierto hacia la tapa)
o = dup(inner); intersect(o, box_mm(0.352, 0.412, -0.019, 0.019, -0.06, -0.012))
cut(o, box_mm(0.3535, 0.4105, -0.0175, 0.0175, -0.07, -0.0135))
part(o, "lipo_battery_compartment", C_FOR, "rPET_int", L_ELB, F + Vector((0, 0, -0.075)),
     desc="Compartimento LiPo 1S / supercapacitor (almacenamiento dual)")

# placa de anclaje de resortes/elasticos de extension
def tray_bottom_at(x, y):
    w, h, zc = sec_at(FORE_K, x)
    a, b = w / 2 - TRAY_OFF, h / 2 - TRAY_OFF
    return zc + b * max(0.0, 1 - (abs(y) / a) ** P_EXP) ** (1 / P_EXP) - 0.002


o = dup(inner); intersect(o, box_mm(0.4100, 0.4130, -0.020, 0.020, 0.0040, tray_bottom_at(0.413, 0.020) - 0.0008))
for yy in (-0.016, -0.008, 0.0, 0.008, 0.016):
    cut(o, cyl(0.0011, 0.01, (0.4115, yy, 0.016), (1, 0, 0), 16))
part(o, "spring_anchor_plate", C_FOR, "rPET_int", L_ELB, F + Vector((0.0, 0.0, 0.045)),
     desc="Placa de anclaje de los elasticos de retorno (extension)")

# base de 5 servos (MG90S) - 3 + 2 en dos filas
SERVO_ROWS = [(0.437, (-0.0145, 0.0, 0.0145)), (0.475, (-0.00725, 0.00725))]
o = dup(inner); intersect(o, box_mm(0.416, 0.494, -0.1, 0.1, -0.0005, 0.0020))
servo_pul = []
for xc, ys in SERVO_ROWS:
    for yy in ys:
        cut(o, box_mm(xc - 0.01145, xc + 0.01145, yy - 0.0062, yy + 0.0062, -0.01, 0.01))
        for xh in (xc - 0.0140, xc + 0.0140):
            cut(o, cyl(0.0009, 0.01, (xh, yy, 0.001), (0, 0, 1), 12))
        servo_pul.append(Vector((xc + 0.0055, yy, 0.016)))
part(o, "five_servo_mount_base", C_FOR, "rPET_int", L_ELB, F + Vector((0, 0, -0.06)),
     desc="Base para 5 micro servos MG90S (4 dedos + flexion pulgar)")
delete(inner)

# tapa distal del antebrazo
o = solid([(0.5120, 0.0570, 0.0470, 0.0), (0.5180, 0.0566, 0.0466, 0.0)])
cut(o, cyl(0.0076, 0.02, (0.515, 0, 0), (1, 0, 0)))
for k in range(4):
    a = math.pi / 4 + k * math.pi / 2
    cut(o, cyl(0.0012, 0.02, (0.515, 0.0145 * math.cos(a), 0.0145 * math.sin(a)), (1, 0, 0), 16))
part(o, "forearm_distal_cap", C_FOR, "rPET_int", L_ELB, F + Vector((0.035, 0, 0)),
     desc="Mamparo distal: soporte de la muneca")

# tubos guia de tendones (x5, PTFE)
DIGITS = ["thumb", "index", "middle", "ring", "pinky"]
tube_ends = {}
# fila 1 (y -14.5, 0, 14.5): menique, medio, indice  | fila 2 (y -7.25, 7.25): anular, pulgar
# los 5 tubos terminan en haz (pentagono) antes del pasa-cables de la muneca
BUNDLE = {"pinky": (-0.0036, 0.0020), "middle": (0.0, 0.0041), "index": (0.0036, 0.0020),
          "ring": (-0.0022, -0.0029), "thumb": (0.0022, -0.0029)}
for d, sp in zip(["pinky", "middle", "index", "ring", "thumb"], servo_pul):
    by, bz = BUNDLE[d]
    if sp.x < 0.46:
        way = [sp, Vector((0.468, sp.y, 0.0200)), Vector((0.492, sp.y * 0.55 + by * 0.45, 0.0135 + bz * 0.5))]
    else:
        way = [sp, Vector((0.492, sp.y * 0.55 + by * 0.45, 0.0085 + bz * 0.5))]
    pts = smooth_path(way + [Vector((0.5073, by, bz))], 8)
    o = curve_mesh(pts, 0.0015, "ttube")
    inner_t = curve_mesh(pts, 0.0009, "ttube_in")
    cut(o, inner_t)
    tube_ends[d] = (pts[0], pts[-1])
    part(o, f"tendon_guide_tube_{d}", C_FOR, "PTFE", L_ELB, F + Vector((0.02, 0, 0.035)),
         desc="Tubo guia PTFE 3/1.8 mm (Bowden) servo -> muneca")

# =============================================================================
#  04 MUNECA
# =============================================================================
W = Vector((0.17, 0, 0))
o = tube(0.0200, 0.0101, 0.5183, 0.5340, (0, 0, 0), (1, 0, 0), 64)
bevel(o, 0.0008, 2, 40)
part(o, "wrist_proximal_body", C_WRI, "rPET_int", L_ELB, W + Vector((-0.01, 0, 0)),
     desc="Cuerpo proximal de la muneca (fijo al antebrazo)")
o = tube(0.0100, 0.0078, 0.5190, 0.5510, (0, 0, 0), (1, 0, 0))
part(o, "wrist_rotation_axis", C_WRI, "acero", L_ELB, W + Vector((0.01, 0, -0.06)),
     desc="Eje hueco de prono-supinacion 20/15.6 mm")
o = tube(0.0070, 0.0055, 0.5080, 0.5545, (0, 0, 0), (1, 0, 0))
part(o, "wrist_cable_passthrough", C_WRI, "PTFE", L_ELB, W + Vector((0.01, 0, -0.09)),
     desc="Pasa-cables/tendones a traves de la muneca")
o = tube(0.0230, 0.0203, 0.5295, 0.5385, (0, 0, 0), (1, 0, 0), 64)
bevel(o, 0.0005, 2, 40)
for k in range(12):
    a = 2 * math.pi * k / 12
    cut(o, box((0.004, 0.0016, 0.0016), (0.534, 0.0231 * math.cos(a), 0.0231 * math.sin(a)),
               Matrix.Rotation(a, 3, 'X')))
part(o, "wrist_locking_ring", C_WRI, "rPET", L_ELB, W + Vector((0.0, 0, 0.06)),
     desc="Anillo de bloqueo con estrias (bloquea la rotacion)")
o = tube(0.0140, 0.0101, 0.5355, 0.5505, (0, 0, 0), (1, 0, 0))
part(o, "wrist_friction_bushing", C_WRI, "joint", L_WRI, W + Vector((0.02, 0, 0.09)),
     desc="Buje de friccion (PET/PTFE) para prono-supinacion")
o = tube(0.0190, 0.0141, 0.5345, 0.5515, (0, 0, 0), (1, 0, 0), 64)
bevel(o, 0.0006, 2, 40)
part(o, "wrist_distal_body", C_WRI, "rPET_int", L_WRI, W + Vector((0.035, 0, 0)),
     desc="Cuerpo distal de la muneca (gira con la mano)")
o = tube(0.0222, 0.0193, 0.5390, 0.5515, (0, 0, 0), (1, 0, 0), 64)
bevel(o, 0.0005, 2, 40)
part(o, "wrist_cosmetic_cover", C_WRI, "rPET", L_WRI, W + Vector((0.035, 0, 0.06)),
     desc="Funda cosmetica de la muneca")

# =============================================================================
#  05 MANO / PALMA  +  PULGAR (marco)
# =============================================================================
H = Vector((0.26, 0, 0))
FINGERS = {
    "index":  dict(y=0.0265, w=0.0165, t=0.0145, L=(0.038, 0.022, 0.019), xm=0.644, rest=(5, 8, 5), rib0=0.604),
    "middle": dict(y=0.0085, w=0.0170, t=0.0150, L=(0.041, 0.025, 0.020), xm=0.646, rest=(8, 12, 6), rib0=0.586),
    "ring":   dict(y=-0.0095, w=0.0160, t=0.0145, L=(0.039, 0.024, 0.019), xm=0.644, rest=(10, 15, 8), rib0=0.586),
    "pinky":  dict(y=-0.0270, w=0.0140, t=0.0130, L=(0.030, 0.018, 0.017), xm=0.641, rest=(14, 18, 10), rib0=0.592),
}
FZ = 0.001
G = 0.0045          # semi-holgura en cada articulacion (barril r=4 mm al centro)

# marco del pulgar
TC = Vector((0.574, 0.022, -0.006))
x_t = Vector((0.80, 0.40, -0.45)).normalized()
v = Vector((0.0, -0.55, -0.80))
pal_t = (v - v.dot(x_t) * x_t).normalized()
z_t = -pal_t
y_t = z_t.cross(x_t).normalized()
R_T = Matrix((x_t, y_t, z_t)).transposed()
M_T = Matrix.Translation(TC) @ R_T.to_4x4()

# palma (hueca)
palm = shell(PALM_K, WALL, cap0=True, name="palm")
cut(palm, cyl(0.0075, 0.01, (0.553, 0, 0), (1, 0, 0)))
for k in range(4):
    a = math.pi / 4 + k * math.pi / 2
    cut(palm, cyl(0.0012, 0.01, (0.553, 0.0145 * math.cos(a), 0.0145 * math.sin(a)), (1, 0, 0), 16))
thumb_open = sphere(0.0140, TC)
for a_op in (0.0, -15.0, -30.0, -45.0):     # ranura barrida por el arco de oposicion
    d_op = Matrix.Rotation(math.radians(a_op), 3, 'X') @ x_t
    union(thumb_open, cyl(0.0140, 0.07, TC + d_op * 0.031, d_op))
cut(palm, thumb_open)
SEAM = -0.003
o = dup(palm); intersect(o, box_mm(0.5, 0.7, -0.1, 0.1, -0.1, SEAM))
part(o, "palm_palmar_grip_surface", C_HAND, "TPU", L_WRI, H + Vector((0, 0, -0.08)),
     desc="Cara palmar de agarre (TPU texturizado)")
cut(palm, box_mm(0.5, 0.7, -0.1, 0.1, -0.1, SEAM + 0.0003))
cov_cut = lambda off: rbox(0.568 - off, 0.616 + off, -0.024 - off, 0.018 + off, 0.006 - off, 0.05, 0.006)
o = dup(palm); intersect(o, cov_cut(0.0))
part(o, "palm_cosmetic_cover", C_HAND, "rPET", L_WRI, H + Vector((0, 0, 0.11)),
     desc="Tapa cosmetica dorsal (acceso a servo del pulgar)")
cut(palm, cov_cut(0.0004))
part(palm, "palm_dorsal", C_HAND, "rPET", L_WRI, H + Vector((0, 0, 0.05)), desc="Palma dorsal estructural hueca")

palm_inner = solid(offset_keys(PALM_K, WALL + 0.0003))
# soportes metacarpianos (x4): costilla interna + bloque distal con chaflan palmar
for fname, f in FINGERS.items():
    rib = dup(palm_inner); intersect(rib, box_mm(f["rib0"], 0.6302, f["y"] - 0.003, f["y"] + 0.003, -0.1, 0.1))
    hh = f["t"] / 2
    xe = f["xm"] - G
    blk = prism([(0.6303, FZ + hh), (xe, FZ + hh), (xe, FZ), (xe - hh * 0.9, FZ - hh), (0.6303, FZ - hh)],
                f["w"], 'XZ', f["y"])
    bevel(blk, 0.0010, 2, 30)
    bridge = box_mm(0.6290, 0.6308, f["y"] - 0.003, f["y"] + 0.003, FZ - hh + 0.001, FZ + hh - 0.001)
    union(rib, bridge, blk)
    cut(rib, cyl(0.0013, 0.06, (0.62, f["y"], FZ - hh + 0.0022), (1, 0, 0), 16))
    part(rib, f"metacarpal_support_{fname}", C_HAND, "rPET_int", L_WRI,
         H + Vector((0.03, f["y"] * 1.6, -0.03)), desc="Soporte metacarpiano / guia de tendon")

# soporte servo del pulgar (placa conformada + pestanas)
o = dup(palm_inner); intersect(o, box_mm(0.556, 0.582, -0.1, 0.1, 0.0018, 0.0038))
cut(o, sphere(0.0135, TC))
tabs = [box_mm(xa, xa + 0.002, -0.0155, 0.0115, 0.0037, 0.0110) for xa in (0.5555, 0.5805)]
for tb_, xa in zip(tabs, (0.5555, 0.5805)):
    for yy in (-0.0105, 0.0065):
        cut(tb_, cyl(0.0009, 0.01, (xa + 0.001, yy, 0.0085), (1, 0, 0), 12))
o = union(o, *tabs)
part(o, "thumb_servo_mount", C_THUMB, "rPET_int", L_WRI, H + Vector((-0.01, 0, 0.075)),
     desc="Montaje del micro servo de oposicion del pulgar")

# soporte metacarpiano del pulgar a 45 deg (collar de la CMC)
o = lathe([(0.0066, -0.0072), (0.0102, -0.0072), (0.0102, -0.0032), (0.0066, -0.0032)], TC, x_t, 48)
part(o, "thumb_metacarpal_support_45deg", C_THUMB, "rPET_int", L_WRI, H + Vector((0.0, 0.06, -0.035)),
     desc="Collar de la articulacion CMC, inclinado 45 deg (oposicion)")
# soporte angular 15-45 deg (cuna con caras a 15 y 45 deg)
wy0, wy1 = 0.010, 0.016
t15, t45 = math.tan(math.radians(15)), math.tan(math.radians(45))
o = prism([(0.556, 0.0016), (0.564, 0.0016), (0.564, -0.0010), (0.560, -0.0010 - 0.004 * t15),
           (0.556, -0.0010 - 0.004 * t15 - 0.004 * t45)], wy1 - wy0, 'XZ', (wy0 + wy1) / 2)
part(o, "thumb_angular_support_15_45", C_THUMB, "rPET_int", L_WRI, H + Vector((0.0, 0.04, -0.06)),
     desc="Cuna de ajuste angular del pulgar 15-45 deg")
delete(palm_inner)

# =============================================================================
#  06 DEDOS  (cadenas MCP -> PIP -> DIP)
# =============================================================================


def phalanx_local(x0, x1, w, t, ch0=True, ch1=True):
    h = t / 2
    c = min(h * 0.9, ((x1 - x0) / 2 - 0.0005) if (ch0 and ch1) else (x1 - x0 - 0.001))
    poly = [(x0, h), (x1, h)]
    poly += [(x1, 0.0), (x1 - c, -h)] if ch1 else [(x1, -h)]
    poly += [(x0 + c, -h), (x0, 0.0)] if ch0 else [(x0, -h)]
    o = prism(poly, w, 'XZ')
    bevel(o, 0.0024, 4, 30)
    return o


def knuckle(w, M, spring_gap=0.0):
    """barril de articulacion (r 4 mm) + pasador de acero; con hueco central para resorte"""
    hw = w / 2 - 0.0005
    if spring_gap > 0:
        k1 = tube(0.0040, 0.00125, -hw, -spring_gap, (0, 0, 0), (0, 1, 0), 40)
        k2 = tube(0.0040, 0.00125, spring_gap, hw, (0, 0, 0), (0, 1, 0), 40)
        bar = join([k1, k2])
    else:
        bar = tube(0.0040, 0.00125, -hw, hw, (0, 0, 0), (0, 1, 0), 40)
    pin = cyl(0.0011, 2 * hw + 0.0012, (0, 0, 0), (0, 1, 0), 20)
    for p in pin.data.polygons:
        p.material_index = 1
    o = join([bar, pin])
    return transform(o, M)


def finger_chain(prefix, base_M, parent_link, w, t, Ls, rest, explode0, spread, coll, mcp_limits=(0, 90),
                 spring=True, first_from_pivot=True):
    """construye una cadena de 3 segmentos con juntas en base_M (pivote MCP)."""
    links = []
    names = ["mcp", "pip", "dip"] if prefix != "thumb" else ["mcp", "ip"]
    segs = ["proximal", "middle", "distal"] if prefix != "thumb" else ["proximal", "distal"]
    hz = -t / 2 + 0.0022
    x = 0.0
    parent = parent_link
    for i, (jn, sg) in enumerate(zip(names, segs)):
        L = Ls[i]
        Mj = base_M @ Matrix.Translation((x, 0, 0))
        lk = make_link(f"{prefix}_{jn}_joint", Mj.to_translation(), parent, Mj.to_3x3(), "revolute",
                       (0, 1, 0), (0, 90) if jn != "mcp" else mcp_limits, rest[i],
                       desc=f"{prefix} {jn.upper()} flexion")
        links.append(lk)
        ex = explode0 + spread * (i * 1.0)
        # articulacion
        o = knuckle(w, Mj, 0.0026 if (spring and jn == "mcp") else 0.0)
        part(o, f"{'finger_' if prefix != 'thumb' else ''}{prefix}_joint_{jn}", coll, "joint", lk,
             ex - spread * 0.45, extra_mats=("acero",), desc=f"Articulacion {jn.upper()}: barril + pasador 2.2 mm")
        if spring and jn == "mcp":
            o = transform(helix((0, 0, 0), (0, 1, 0), 0.0026, 0.00033, 0.0040, 5.0), Mj)
            part(o, f"{'finger_' if prefix != 'thumb' else ''}{prefix}_return_spring", coll, "resorte", lk,
                 ex - spread * 0.45 + Vector((0, 0, 0.03)), desc="Resorte de torsion de retorno (extension)")
        # falange
        last = (i == len(segs) - 1)
        x1 = (L - G) if not last else (L - 0.0094)
        o = phalanx_local(G, x1, w, t, ch0=True, ch1=not last)
        ch = cyl(0.0013 if i == 0 else 0.0011, L + 0.02, (L / 2, 0, hz), (1, 0, 0), 16)
        cut(o, ch)
        if last:
            cut(o, cyl(0.00085, w + 0.01, (x1 - 0.004, 0, hz), (0, 1, 0), 12))
        transform(o, Mj)
        part(o, f"{'finger_' if prefix != 'thumb' else ''}{prefix}_phalanx_{sg}", coll, "rPET", lk, ex,
             desc=f"Falange {sg} (rPET), canal de tendon palmar")
        if i == 0 and prefix != "thumb":
            sl = tube(0.00125, 0.0007, G + 0.0002, x1 - 0.0002, (0, 0, hz), (1, 0, 0), 16)
            part(transform(sl, Mj), f"finger_{prefix}_tendon_channel", coll, "PTFE", lk,
                 ex + Vector((0, 0, -0.025)), desc="Camisa PTFE del canal de tendon")
        if last:
            an = cyl(0.0008, w - 0.0012, (x1 - 0.004, 0, hz), (0, 1, 0), 12)
            part(transform(an, Mj), f"{'finger_' if prefix != 'thumb' else ''}{prefix}_distal_tendon_anchor",
                 coll, "acero", lk, ex + Vector((0, 0, -0.02)), desc="Pasador de anclaje distal del tendon")
            pad = rbox(x1 + 0.0003, L, -w / 2 - 0.0003, w / 2 + 0.0003, -t / 2 - 0.0003, t / 2 + 0.0003,
                       min(0.0052, t / 2), 5)
            part(transform(pad, Mj), f"{'finger_' if prefix != 'thumb' else ''}{prefix}_fingertip_pad", coll,
                 "TPU", lk, ex + spread * 0.6, desc="Yema TPU de alta friccion")
        parent = lk
        x += L
    return links


for fname, f in FINGERS.items():
    M0 = Matrix.Translation((f["xm"], f["y"], FZ))
    ex0 = H + Vector((0.075, f["y"] * 2.8, 0.0))
    finger_chain(fname, M0, L_WRI, f["w"], f["t"], f["L"], f["rest"], ex0, Vector((0.034, 0, 0)), C_FING)

# =============================================================================
#  07 PULGAR  (CMC 2 GDL -> MCP -> IP)
# =============================================================================
L_OPP = make_link("thumb_cmc_opposition_joint", TC, L_WRI, None, "revolute", (-1, 0, 0), (0, 45), 0.0, 'X',
                  desc="CMC: arco de oposicion 0-45 deg (rest = 15 deg anatomico)")
L_CMC = make_link("thumb_cmc_flexion_joint", TC, L_OPP, R_T, "revolute", (0, 1, 0), (0, 30), 0.0,
                  desc="CMC: flexion")
TEX = H + Vector((0.01, 0.07, -0.05))
o = sphere(0.0068, TC)
part(o, "thumb_joint_cmc", C_THUMB, "joint", L_CMC, TEX, desc="Rotula de la articulacion carpometacarpiana")
L_MET = 0.042
o = phalanx_local(0.0075, L_MET - G, 0.019, 0.016, ch0=False, ch1=True)
cut(o, cyl(0.0013, 0.08, (0.02, 0, -0.008 + 0.0022), (1, 0, 0), 16))
part(transform(o, M_T), "thumb_metacarpal", C_THUMB, "rPET", L_CMC, TEX + x_t * 0.02,
     desc="Metacarpiano del pulgar (segmento proximal movil)")
finger_chain("thumb", M_T @ Matrix.Translation((L_MET, 0, 0)), L_CMC, 0.018, 0.0155, (0.031, 0.026), (8, 10),
             TEX + x_t * 0.05, x_t * 0.028, C_THUMB, mcp_limits=(0, 60), spring=True)

# =============================================================================
#  08 TENDONES VIRTUALES (nylon) - waypoints enganchados a cada eslabon
# =============================================================================
bpy.context.view_layer.update()


def wpt(link, local):
    return link.matrix_world @ Vector(local)


TENDONS = {}
for d in DIGITS:
    st, en = tube_ends[d]
    pts = [(st, L_ELB), (en, L_ELB), (Vector((0.5450, 0, 0)), L_WRI)]
    if d == "thumb":
        pts += [(Vector((0.565, 0.006, -0.006)), L_WRI), (TC + Vector((0, -0.004, -0.008)), L_WRI)]
        hz = -0.008 + 0.0022
        lk = LINKS["thumb_mcp_joint"]
        pts += [(wpt(LINKS["thumb_cmc_flexion_joint"], (0.02, 0, hz)), LINKS["thumb_cmc_flexion_joint"])]
        pts += [(wpt(LINKS["thumb_mcp_joint"], (0.0, 0, -0.0078 + 0.0022)), LINKS["thumb_cmc_flexion_joint"])]
        pts += [(wpt(LINKS["thumb_ip_joint"], (0.0, 0, -0.0078 + 0.0022)), LINKS["thumb_mcp_joint"])]
        pts += [(wpt(LINKS["thumb_ip_joint"], (0.026 - 0.0094 - 0.004, 0, -0.0078 + 0.0022)), LINKS["thumb_ip_joint"])]
    else:
        f = FINGERS[d]
        hz = -f["t"] / 2 + 0.0022
        pts += [(Vector((0.586, f["y"], FZ + hz)), L_WRI)]
        for jn, plink in (("mcp", L_WRI), ("pip", LINKS[f"{d}_mcp_joint"]), ("dip", LINKS[f"{d}_pip_joint"])):
            pts += [(wpt(LINKS[f"{d}_{jn}_joint"], (0, 0, hz)), plink)]
        pts += [(wpt(LINKS[f"{d}_dip_joint"], (f["L"][2] - 0.0094 - 0.004, 0, hz)), LINKS[f"{d}_dip_joint"])]
    cu = bpy.data.curves.new(f"tendon_{d}", 'CURVE')
    cu.dimensions, cu.bevel_depth = '3D', 0.0004
    sp = cu.splines.new('POLY')
    sp.points.add(len(pts) - 1)
    for p, (co, _) in zip(sp.points, pts):
        p.co = (co.x, co.y, co.z, 1.0)
    cu.materials.append(MAT["nylon"])
    ob = bpy.data.objects.new(f"tendon_virtual_{d}", cu)
    C_TEND.objects.link(ob)
    ob.parent = L_ROOT
    ob.hide_render = True
    ob["descripcion"] = "Tendon virtual de nylon (waypoints con hooks por eslabon) para simulacion de traccion"
    for i, (co, lk) in enumerate(pts):
        hm = ob.modifiers.new(f"hook_{i}_{lk.name}", 'HOOK')
        hm.object = lk
        hm.vertex_indices_set([i])
        hm.matrix_inverse = lk.matrix_world.inverted()
    TENDONS[d] = [[round(c, 5) for c in co] + [lk.name] for co, lk in pts]

# =============================================================================
#  POSE DE REPOSO, EXPLOSION, LIMPIEZA
# =============================================================================
bpy.context.view_layer.update()
for fname, f in FINGERS.items():
    for jn, r in zip(("mcp", "pip", "dip"), f["rest"]):
        LINKS[f"{fname}_{jn}_joint"].rotation_euler[1] = math.radians(r)
for jn, r in zip(("mcp", "ip"), (8, 10)):
    LINKS[f"thumb_{jn}_joint"].rotation_euler[1] = math.radians(r)

# ---- explosion de la mano en cuadricula: una fila por dedo, con los dedos estirados (explosion = 1)
# (con las juntas en 0 la transformacion padre es la identidad -> los deltas son vectores de mundo)
PE = {p["obj"].name: p for p in PARTS}
hand_names = {o.name for c in (C_HAND, C_FING, C_THUMB) for o in c.objects}
laid = set()


def place(n, x, y, z=0.0):
    PE[n]["explode"] = Vector((x, y, z)) - PE[n]["obj"].location
    laid.add(n)


def lay_row(names, x0, y, gap=0.009):
    centers, x = {}, x0
    for n in names:
        sx = PE[n]["obj"]["size_mm"][0] / 1000
        place(n, x + sx / 2, y)
        centers[n] = x + sx / 2
        x += sx + gap
    return centers


HX = 0.815                                     # columna de la palma
place("palm_cosmetic_cover", HX, 0.088)
place("palm_dorsal", HX, 0.0)
place("palm_palmar_grip_surface", HX, -0.092)
ROWS = {"index": 0.070, "middle": 0.025, "ring": -0.020, "pinky": -0.065}
SUB = -0.021                                   # sub-fila de accesorios bajo cada pieza
for f, yr in ROWS.items():
    place(f"metacarpal_support_{f}", 0.885, yr)
    cs = lay_row([f"finger_{f}_joint_mcp", f"finger_{f}_phalanx_proximal", f"finger_{f}_joint_pip",
                  f"finger_{f}_phalanx_middle", f"finger_{f}_joint_dip", f"finger_{f}_phalanx_distal",
                  f"finger_{f}_fingertip_pad"], 0.925, yr)
    place(f"finger_{f}_return_spring", cs[f"finger_{f}_joint_mcp"], yr + SUB)
    place(f"finger_{f}_tendon_channel", cs[f"finger_{f}_phalanx_proximal"], yr + SUB)
    place(f"finger_{f}_distal_tendon_anchor", cs[f"finger_{f}_phalanx_distal"], yr + SUB)
yt = 0.150
cs = lay_row(["thumb_servo_mount", "thumb_metacarpal_support_45deg", "thumb_joint_cmc", "thumb_metacarpal",
              "thumb_joint_mcp", "thumb_phalanx_proximal", "thumb_joint_ip", "thumb_phalanx_distal",
              "thumb_fingertip_pad"], 0.785, yt)
place("thumb_angular_support_15_45", cs["thumb_metacarpal_support_45deg"], yt + SUB - 0.004)
place("thumb_return_spring", cs["thumb_joint_mcp"], yt + SUB)
place("thumb_distal_tendon_anchor", cs["thumb_phalanx_distal"], yt + SUB)
missing = hand_names - laid
if missing:
    print("[LRM] AVISO piezas de mano sin posicion en la cuadricula:", sorted(missing))

# =============================================================================
#  PANEL DE CONTROL: deslizadores en LRM_CONTROL -> drivers en juntas y piezas
#  (expresiones simples: Blender las evalua sin necesitar permitir scripts)
# =============================================================================
CTRL = bpy.data.objects.new("LRM_CONTROL", None)
scene.collection.objects.link(CTRL)
CTRL.empty_display_type = 'SPHERE'
CTRL.empty_display_size = 0.012
CTRL.location = (0.0, -0.10, 0.08)
CTRL.hide_render = True
CTRL["descripcion"] = "Controles de la protesis LRM. Panel: vista 3D > tecla N > pestana 'LRM Protesis'"

CONTROLS = [  # nombre, min, max, subtipo, descripcion
    ("codo", 0.0, math.radians(135), 'ANGLE', "Flexion del codo 0-135 deg"),
    ("muneca", -math.radians(90), math.radians(90), 'ANGLE', "Prono-supinacion de la muneca +-90 deg"),
    ("cierre_mano", 0.0, 1.0, 'FACTOR', "Agarre cilindrico: cierra los 4 dedos y opone el pulgar"),
    ("pinza", 0.0, 1.0, 'FACTOR', "Pinza de precision pulgar-indice"),
    ("indice", 0.0, 1.0, 'FACTOR', "Flexion individual del indice"),
    ("medio", 0.0, 1.0, 'FACTOR', "Flexion individual del dedo medio"),
    ("anular", 0.0, 1.0, 'FACTOR', "Flexion individual del anular"),
    ("menique", 0.0, 1.0, 'FACTOR', "Flexion individual del menique"),
    ("pulgar_flexion", 0.0, 1.0, 'FACTOR', "Flexion del pulgar (CMC + MCP + IP)"),
    ("pulgar_oposicion", 0.0, 1.0, 'FACTOR', "Arco de oposicion del pulgar 0-45 deg"),
    ("explosion", 0.0, 1.0, 'FACTOR', "Vista explosionada (0 = ensamblada, 1 = explosionada)"),
    ("panel_solar", 0.0, 1.0, 'FACTOR', "Retirar el panel solar de su canal"),
]
for name, mn, mx, sub, desc in CONTROLS:
    CTRL[name] = 0.0
    CTRL.id_properties_ui(name).update(min=mn, max=mx, soft_min=mn, soft_max=mx, subtype=sub,
                                       description=desc, default=0.0)

NOT_SIMPLE = []


def drive(obj, path, idx, expr, props):
    fc = obj.driver_add(path, idx)
    d = fc.driver
    d.type = 'SCRIPTED'
    for vn, prop in props.items():
        v = d.variables.new()
        v.name, v.type = vn, 'SINGLE_PROP'
        v.targets[0].id_type = 'OBJECT'
        v.targets[0].id = CTRL
        v.targets[0].data_path = f'["{prop}"]'
    d.expression = expr
    if not d.is_simple_expression:
        NOT_SIMPLE.append(f"{obj.name}.{path}[{idx}]: {expr}")


def f6(x):
    return f"{x:.6f}"


# brazo
drive(L_ELB, "rotation_euler", 1, "-c", {"c": "codo"})
drive(L_WRI, "rotation_euler", 0, "w", {"w": "muneca"})
# dedos: angulo = max(reposo*(1-explosion), objetivo*min(1, cierre + dedo + pinza))
FLEX_TARGET = {"mcp": 80, "pip": 90, "dip": 65}
CTRL_NAME = {"index": "indice", "middle": "medio", "ring": "anular", "pinky": "menique"}
for fname, f in FINGERS.items():
    for jn, r in zip(("mcp", "pip", "dip"), f["rest"]):
        R, T = f6(math.radians(r)), f6(math.radians(FLEX_TARGET[jn]))
        if fname == "index":
            # en la pinza el indice se flexiona ~55% para encontrar la yema del pulgar
            drive(LINKS[f"{fname}_{jn}_joint"], "rotation_euler", 1, f"max({R}*(1-e), {T}*min(1, c+d+0.55*p))",
                  {"e": "explosion", "c": "cierre_mano", "d": CTRL_NAME[fname], "p": "pinza"})
        else:
            drive(LINKS[f"{fname}_{jn}_joint"], "rotation_euler", 1, f"max({R}*(1-e), {T}*min(1, c+d))",
                  {"e": "explosion", "c": "cierre_mano", "d": CTRL_NAME[fname]})
# pulgar
TH = {"e": "explosion", "c": "cierre_mano", "p": "pinza", "t": "pulgar_flexion", "o": "pulgar_oposicion"}
drive(L_OPP, "rotation_euler", 0, f"-{f6(math.radians(45))}*min(1, o+c+p)", {k: TH[k] for k in "ocp"})
drive(L_CMC, "rotation_euler", 1, f"{f6(math.radians(30))}*min(1, t+0.6*c+0.5*p)", {k: TH[k] for k in "tcp"})
drive(LINKS["thumb_mcp_joint"], "rotation_euler", 1,
      f"max({f6(math.radians(8))}*(1-e), {f6(math.radians(55))}*min(1, t+0.6*c+0.6*p))", {k: TH[k] for k in "etcp"})
drive(LINKS["thumb_ip_joint"], "rotation_euler", 1,
      f"max({f6(math.radians(10))}*(1-e), {f6(math.radians(80))}*min(1, t+0.7*c+0.5*p))", {k: TH[k] for k in "etcp"})
# explosion + panel solar (delta de cada pieza)
for p in PARTS:
    o = p["obj"]
    for i in range(3):
        ex = p["explode"][i]
        lf = p["lift"][0][i] if p["lift"] is not None else 0.0
        if abs(ex) < 1e-7 and abs(lf) < 1e-7:
            continue
        expr = f"e*{f6(ex)}" + (f" + s*{f6(lf)}" if abs(lf) > 1e-7 else "")
        drive(o, "delta_location", i, expr, {"e": "explosion", "s": "panel_solar"} if abs(lf) > 1e-7
              else {"e": "explosion"})
    if p["lift"] is not None:
        drive(o, "delta_rotation_euler", 1, f"s*{f6(p['lift'][1])}*(1-e)", {"s": "panel_solar", "e": "explosion"})
if NOT_SIMPLE:
    print("[LRM] AVISO drivers que requieren permitir scripts:", NOT_SIMPLE[:5])
print(f"[LRM] controles: {len(CONTROLS)} deslizadores, drivers simples OK={not NOT_SIMPLE}")

# panel de la barra lateral (se registra solo al abrir el .blend si los scripts estan permitidos)
_panel_src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lrm_panel.py"), encoding="utf-8").read()
txt = bpy.data.texts.new("lrm_panel.py")
txt.write(_panel_src)
txt.use_module = True
scene.frame_start, scene.frame_end = 1, 250
scene.frame_set(1)

scene["lrm_info"] = ("LRM ROBOTICS protesis transhumeral V1. Movimiento: objeto LRM_CONTROL (deslizadores) o panel "
                     "'LRM Protesis' (tecla N en la vista 3D). Juntas = Empties en 00_CINEMATICA_juntas.")

# ---------------------------------------------------------------- reporte cinematico
os.makedirs(os.path.dirname(KIN_OUT), exist_ok=True)
bpy.context.view_layer.update()
kin = {"units": "m (mostrar en mm)", "up_axis": "Z", "forward": "+X (brazo derecho, palma -Z)",
       "origin": "cara proximal del socket", "joints": [], "parts": [], "tendons": TENDONS}
for name, e in LINKS.items():
    mw = e.matrix_world
    kin["joints"].append({"name": name, "parent": e.parent.name if e.parent else None, "type": e["joint_type"],
                          "pivot_m": [round(c, 5) for c in mw.to_translation()],
                          "axis_world": [round(c, 4) for c in (mw.to_3x3() @ Vector(e.get("axis_local", (0, 0, 0))))],
                          "limits_deg": list(e.get("limit_deg", [])), "rest_deg": e.get("rest_deg", 0.0),
                          "desc": e["descripcion"]})
for p in PARTS:
    o = p["obj"]
    kin["parts"].append({"name": o.name, "link": o["kin_link"], "material": o["material"],
                         "physics": o["physics"], "size_mm": list(o["size_mm"]), "desc": o["descripcion"]})
with open(KIN_OUT, "w", encoding="utf-8") as fh:
    json.dump(kin, fh, indent=1, ensure_ascii=False)

print(f"[LRM] piezas: {len(PARTS)}  juntas: {len(LINKS)}  tendones: {len(TENDONS)}")

# ---------------------------------------------------------------- escenario
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "lrm_escenario.py"), encoding="utf-8").read())

for c in list(TMP.objects):
    delete(c)
bpy.data.collections.remove(TMP)
for me in list(bpy.data.meshes):
    if me.users == 0:
        bpy.data.meshes.remove(me)

# vista inicial al abrir: enfocada en la protesis, con colores de material y barra lateral (N) abierta
view_dir = Vector((-0.32, 1.0, -0.55)).normalized()
for scr in bpy.data.screens:
    for area in scr.areas:
        if area.type != 'VIEW_3D':
            continue
        for sp in area.spaces:
            if sp.type == 'VIEW_3D':
                sp.show_region_ui = True
                sp.shading.type = 'SOLID'
                sp.shading.color_type = 'MATERIAL'
                sp.clip_start, sp.clip_end = 0.005, 100
                r3 = sp.region_3d
                r3.view_perspective = 'PERSP'
                r3.view_location = (0.36, 0.0, 0.0)
                r3.view_distance = 1.15
                r3.view_rotation = view_dir.to_track_quat('-Z', 'Y')
# el escenario (piso, luces, camaras, aros) no estorba en la vista de trabajo; sigue en los renders.
# Para verlo: Outliner > ojo de la coleccion ESCENARIO_LRM, o Alt+H en la vista 3D.
for o in bpy.data.collections["ESCENARIO_LRM"].objects:
    o.hide_set(True)

bpy.ops.wm.save_as_mainfile(filepath=BLEND_OUT)
print("[LRM] guardado:", BLEND_OUT)
