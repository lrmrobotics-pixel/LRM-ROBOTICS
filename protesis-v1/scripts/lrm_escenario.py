# -*- coding: utf-8 -*-
"""Escenario tecnologico LRM: laboratorio oscuro, pedestal con anillo de luz, rejilla en el piso,
paneles de luz en el fondo, iluminacion de estudio y 4 camaras.
Se ejecuta desde lrm_protesis_v1.py (comparte sus utilidades)."""

ESC = get_coll("ESCENARIO_LRM")
CX = 0.38           # centro del pedestal (bajo el antebrazo)
FLOOR_Z = -0.29


def to_esc(o, name, mat=None):
    for c in list(o.users_collection):
        c.objects.unlink(o)
    ESC.objects.link(o)
    o.name = name
    if mat is not None:
        o.data.materials.clear()
        o.data.materials.append(mat)
        o.data.polygons.foreach_set("material_index", [0] * len(o.data.polygons))
    return o


def emission_mat(name, color, strength):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    em = nt.nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = (*color, 1)
    em.inputs["Strength"].default_value = strength
    nt.links.new(em.outputs[0], out.inputs[0])
    return m


# ------------------------------------------------------------------ mundo
world = bpy.data.worlds.new("LRM_lab")
scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes["Background"]
bg.inputs["Color"].default_value = (0.004, 0.007, 0.013, 1)
bg.inputs["Strength"].default_value = 1.0

# ------------------------------------------------------------------ piso con rejilla luminosa
fm = bpy.data.materials.new("Piso_lab_rejilla")
fm.use_nodes = True
nt = fm.node_tree
nd, lk = nt.nodes, nt.links
b = nd["Principled BSDF"]
b.inputs["Base Color"].default_value = (0.010, 0.012, 0.016, 1)
b.inputs["Roughness"].default_value = 0.45
b.inputs["Metallic"].default_value = 0.15
tc = nd.new("ShaderNodeTexCoord")
sep = nd.new("ShaderNodeSeparateXYZ")
lk.new(tc.outputs["Object"], sep.inputs[0])


def grid_line(socket, scale, width):
    mu = nd.new("ShaderNodeMath"); mu.operation = 'MULTIPLY'; mu.inputs[1].default_value = scale
    fr = nd.new("ShaderNodeMath"); fr.operation = 'FRACT'
    lt = nd.new("ShaderNodeMath"); lt.operation = 'LESS_THAN'; lt.inputs[1].default_value = width
    lk.new(socket, mu.inputs[0]); lk.new(mu.outputs[0], fr.inputs[0]); lk.new(fr.outputs[0], lt.inputs[0])
    return lt.outputs[0]


gx = grid_line(sep.outputs["X"], 10.0, 0.012)
gy = grid_line(sep.outputs["Y"], 10.0, 0.012)
mx = nd.new("ShaderNodeMath"); mx.operation = 'MAXIMUM'
lk.new(gx, mx.inputs[0]); lk.new(gy, mx.inputs[1])
vl = nd.new("ShaderNodeVectorMath"); vl.operation = 'LENGTH'
lk.new(tc.outputs["Object"], vl.inputs[0])
fade = nd.new("ShaderNodeMapRange")
fade.inputs["From Min"].default_value = 0.3
fade.inputs["From Max"].default_value = 2.6
fade.inputs["To Min"].default_value = 1.0
fade.inputs["To Max"].default_value = 0.0
lk.new(vl.outputs["Value"], fade.inputs["Value"])
mf = nd.new("ShaderNodeMath"); mf.operation = 'MULTIPLY'
lk.new(mx.outputs[0], mf.inputs[0]); lk.new(fade.outputs[0], mf.inputs[1])
ms = nd.new("ShaderNodeMath"); ms.operation = 'MULTIPLY'; ms.inputs[1].default_value = 0.7
lk.new(mf.outputs[0], ms.inputs[0])
b.inputs["Emission Color"].default_value = (0.10, 0.75, 0.85, 1)
lk.new(ms.outputs[0], b.inputs["Emission Strength"])

floor = box_mm(CX - 4, CX + 4, -4, 4, FLOOR_Z - 0.01, FLOOR_Z)
floor.data.transform(Matrix.Translation((-CX, 0, -FLOOR_Z)))
floor.location = (CX, 0, FLOOR_Z)
to_esc(floor, "piso_laboratorio", fm)

# ------------------------------------------------------------------ pedestal + anillos de luz
pm = principled("Pedestal_satinado", (0.03, 0.035, 0.042), rough=0.32, metal=0.6)
ped = cyl(0.42, 0.04, (CX, 0, FLOOR_Z + 0.02), (0, 0, 1), 128)
bevel(ped, 0.006, 3, 30)
to_esc(ped, "pedestal", pm)
ped2 = cyl(0.36, 0.012, (CX, 0, FLOOR_Z + 0.046), (0, 0, 1), 128)
bevel(ped2, 0.003, 3, 30)
to_esc(ped2, "pedestal_superior", pm)
ring_cyan = emission_mat("Luz_anillo_cian", (0.15, 0.85, 1.0), 9.0)
ring_teal = emission_mat("Luz_anillo_verde", (0.2, 1.0, 0.65), 5.0)


def torus(R, r, center, segs=160, name="torus"):
    prof = [(R + r * math.cos(2 * math.pi * k / 16), r * math.sin(2 * math.pi * k / 16)) for k in range(16)]
    return lathe(prof, center, (0, 0, 1), segs, name)


to_esc(torus(0.425, 0.0035, (CX, 0, FLOOR_Z + 0.041)), "anillo_luz_pedestal", ring_cyan)
to_esc(torus(0.362, 0.0018, (CX, 0, FLOOR_Z + 0.0525)), "anillo_luz_interior", ring_teal)
# arcos holograficos alrededor de la protesis
holo = emission_mat("Holograma_cian", (0.2, 0.8, 1.0), 2.2)
for i, (R, zz) in enumerate(((0.50, 0.00), (0.55, 0.10))):
    prof = [(R + 0.0012 * math.cos(2 * math.pi * k / 8), 0.0012 * math.sin(2 * math.pi * k / 8)) for k in range(8)]
    t = lathe(prof, (CX, 0, zz - 0.06), (0, 0, 1), 200)
    cut(t, box_mm(CX - 1, CX + 1, -1, -0.15 - 0.1 * i, -1, 1))
    to_esc(t, f"arco_holografico_{i}", holo)

# ------------------------------------------------------------------ fondo: muro con panel hexagonal y tiras de luz
wm = bpy.data.materials.new("Muro_panel_tech")
wm.use_nodes = True
nt = wm.node_tree
nd, lk = nt.nodes, nt.links
b = nd["Principled BSDF"]
b.inputs["Base Color"].default_value = (0.012, 0.016, 0.024, 1)
b.inputs["Roughness"].default_value = 0.45
tc = nd.new("ShaderNodeTexCoord")
vo = nd.new("ShaderNodeTexVoronoi")
vo.feature = 'DISTANCE_TO_EDGE'
vo.inputs["Scale"].default_value = 7.0
vo.inputs["Randomness"].default_value = 0.0
lk.new(tc.outputs["Object"], vo.inputs["Vector"])
cr = nd.new("ShaderNodeValToRGB")
cr.color_ramp.elements[0].position = 0.0
cr.color_ramp.elements[0].color = (1, 1, 1, 1)
cr.color_ramp.elements[1].position = 0.035
cr.color_ramp.elements[1].color = (0, 0, 0, 1)
lk.new(vo.outputs["Distance"], cr.inputs["Fac"])
gz = nd.new("ShaderNodeSeparateXYZ")
lk.new(tc.outputs["Object"], gz.inputs[0])
gr = nd.new("ShaderNodeMapRange")
gr.inputs["From Min"].default_value = -0.2
gr.inputs["From Max"].default_value = 1.3
gr.inputs["To Min"].default_value = 0.35
gr.inputs["To Max"].default_value = 0.05
lk.new(gz.outputs["Z"], gr.inputs["Value"])
mu = nd.new("ShaderNodeMath"); mu.operation = 'MULTIPLY'
lk.new(cr.outputs["Color"], mu.inputs[0]); lk.new(gr.outputs[0], mu.inputs[1])
b.inputs["Emission Color"].default_value = (0.08, 0.45, 0.70, 1)
lk.new(mu.outputs[0], b.inputs["Emission Strength"])
wall = box_mm(CX - 2.6, CX + 2.6, 1.30, 1.32, FLOOR_Z, FLOOR_Z + 2.2)
wall.data.transform(Matrix.Translation((-CX, -1.31, -(FLOOR_Z + 0.9))))
wall.location = (CX, 1.31, FLOOR_Z + 0.9)
to_esc(wall, "muro_hexagonal", wm)

strip_c = emission_mat("Tira_luz_cian", (0.25, 0.85, 1.0), 6.0)
strip_g = emission_mat("Tira_luz_verde", (0.25, 1.0, 0.7), 4.0)
for i, xx in enumerate((-0.95, -0.55, 1.30, 1.70)):
    s = box_mm(CX + xx - 0.008, CX + xx + 0.008, 1.24, 1.26, FLOOR_Z + 0.05, FLOOR_Z + 1.9)
    to_esc(s, f"tira_luz_{i}", strip_c if i % 2 == 0 else strip_g)
s = box_mm(CX - 2.4, CX + 2.4, 1.24, 1.26, FLOOR_Z + 0.04, FLOOR_Z + 0.048)
to_esc(s, "tira_luz_zocalo", strip_c)

# ------------------------------------------------------------------ luces de estudio
def area(name, loc, target, size, energy, color=(1, 1, 1), sx=None):
    ld = bpy.data.lights.new(name, 'AREA')
    ld.energy = energy
    ld.color = color
    ld.shape = 'RECTANGLE'
    ld.size = size
    ld.size_y = sx if sx else size
    lo = bpy.data.objects.new(name, ld)
    ESC.objects.link(lo)
    lo.location = loc
    d = Vector(target) - Vector(loc)
    lo.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    return lo


area("luz_principal", (0.05, -1.05, 1.05), (0.38, 0, 0), 1.2, 75)
area("luz_cenital", (0.38, 0.05, 1.25), (0.38, 0, 0), 1.4, 45, sx=0.35)
area("luz_contra_cian", (0.55, 1.0, 0.55), (0.38, 0, 0), 0.7, 120, (0.35, 0.85, 1.0))
area("luz_relleno_azul", (1.45, -0.55, 0.15), (0.38, 0, 0), 0.6, 28, (0.45, 0.6, 1.0))
area("luz_borde_verde", (-0.55, 0.65, 0.35), (0.3, 0, 0), 0.5, 50, (0.4, 1.0, 0.75))

# ------------------------------------------------------------------ camaras
def camera(name, loc, target, lens=50):
    cd = bpy.data.cameras.new(name)
    cd.lens = lens
    cd.clip_start, cd.clip_end = 0.01, 50
    cd.dof.use_dof = False
    co = bpy.data.objects.new(name, cd)
    ESC.objects.link(co)
    co.location = loc
    d = Vector(target) - Vector(loc)
    co.rotation_euler = d.to_track_quat('-Z', 'Y').to_euler()
    co["target"] = list(target)
    return co


cam_hero = camera("CAM_ensamblada_3_4", (0.70, -1.12, 0.44), (0.375, 0.0, -0.005), 50)
camera("CAM_explosionada", (0.44, -1.95, 0.80), (0.44, 0.0, -0.01), 42)
camera("CAM_mano_explosionada", (0.99, -0.52, 0.30), (0.935, 0.02, -0.005), 50)
camera("CAM_panel_solar", (0.29, -0.40, 0.34), (0.43, 0.0, 0.035), 50)
scene.camera = cam_hero

# ------------------------------------------------------------------ render
scene.render.engine = 'CYCLES'
cy = scene.cycles
cy.device = 'CPU'
cy.samples = 160
cy.use_adaptive_sampling = True
cy.adaptive_threshold = 0.015
cy.use_denoising = True
cy.max_bounces = 10
cy.diffuse_bounces = 3
cy.glossy_bounces = 4
cy.transmission_bounces = 10
cy.transparent_max_bounces = 8
cy.caustics_reflective = False
cy.caustics_refractive = False
cy.blur_glossy = 1.0
scene.render.resolution_x, scene.render.resolution_y = 1920, 1080
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = 'PNG'
scene.view_settings.view_transform = 'AgX'
try:
    scene.view_settings.look = 'AgX - Medium High Contrast'
except Exception:
    pass
scene.view_settings.exposure = -0.2
