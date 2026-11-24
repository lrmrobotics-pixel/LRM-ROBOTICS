# -*- coding: utf-8 -*-
"""Panel 'LRM Protesis' (vista 3D > tecla N). Viaja dentro del .blend como texto registrado."""
import bpy

CTRL_NAME = "LRM_CONTROL"
PROPS = ["codo", "muneca", "cierre_mano", "pinza", "indice", "medio", "anular", "menique",
         "pulgar_flexion", "pulgar_oposicion", "explosion", "panel_solar"]
R = 0.0174533
POSES = {
    "REPOSO": ("Reposo", {}),
    "AGARRE": ("Agarre cilindrico", {"cierre_mano": 1.0}),
    "PINZA": ("Pinza", {"pinza": 1.0}),
    "SENALAR": ("Senalar", {"medio": 1.0, "anular": 1.0, "menique": 1.0, "pulgar_flexion": 0.8,
                            "pulgar_oposicion": 0.6}),
    "PULGAR_ARRIBA": ("Pulgar arriba", {"indice": 1.0, "medio": 1.0, "anular": 1.0, "menique": 1.0,
                                        "codo": 90 * R, "muneca": 90 * R}),
    "SALUDO": ("Saludo", {"codo": 115 * R, "muneca": -20 * R}),
    "BEBER": ("Sostener vaso", {"codo": 95 * R, "muneca": 85 * R, "cierre_mano": 0.75}),
    "EXPLOSION": ("Vista explosionada", {"explosion": 1.0}),
    "SOLAR": ("Retirar panel solar", {"panel_solar": 1.0}),
}
# demo: (frame, pose)
DEMO = [(1, {}), (35, {"codo": 90 * R}), (70, {"codo": 90 * R, "cierre_mano": 1.0}),
        (95, {"codo": 90 * R}), (125, {"codo": 90 * R, "pinza": 1.0}), (150, {"codo": 90 * R}),
        (180, {"codo": 90 * R, "muneca": 85 * R, "cierre_mano": 0.75}), (205, {"codo": 90 * R}), (250, {})]


def ctrl():
    return bpy.data.objects.get(CTRL_NAME)


def apply(context, values):
    c = ctrl()
    for p in PROPS:
        c[p] = float(values.get(p, 0.0))
    c.update_tag()
    context.scene.frame_set(context.scene.frame_current)


class LRM_OT_pose(bpy.types.Operator):
    bl_idname = "lrm.pose"
    bl_label = "Aplicar pose"
    bl_description = "Lleva la protesis a una pose predefinida"
    bl_options = {'REGISTER', 'UNDO'}
    pose: bpy.props.EnumProperty(items=[(k, v[0], "") for k, v in POSES.items()])

    def execute(self, context):
        apply(context, POSES[self.pose][1])
        return {'FINISHED'}


class LRM_OT_keyframe(bpy.types.Operator):
    bl_idname = "lrm.keyframe"
    bl_label = "Guardar pose en este frame"
    bl_description = "Inserta un keyframe con todos los controles en el frame actual (para animar)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        c = ctrl()
        for p in PROPS:
            c.keyframe_insert(f'["{p}"]', frame=context.scene.frame_current)
        return {'FINISHED'}


class LRM_OT_clear(bpy.types.Operator):
    bl_idname = "lrm.clear_anim"
    bl_label = "Borrar animacion"
    bl_description = "Elimina los keyframes de los controles"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        ctrl().animation_data_clear()
        apply(context, {})
        return {'FINISHED'}


class LRM_OT_demo(bpy.types.Operator):
    bl_idname = "lrm.demo"
    bl_label = "Crear animacion demo"
    bl_description = "Codo, agarre cilindrico, pinza y sostener vaso (frames 1-250). Luego pulsa Espacio"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        c = ctrl()
        c.animation_data_clear()
        for fr, vals in DEMO:
            for p in PROPS:
                c[p] = float(vals.get(p, 0.0))
                c.keyframe_insert(f'["{p}"]', frame=fr)
        context.scene.frame_start, context.scene.frame_end = 1, 250
        context.scene.frame_set(1)
        return {'FINISHED'}


class LRM_PT_panel(bpy.types.Panel):
    bl_label = "LRM Protesis - Control"
    bl_idname = "LRM_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "LRM Protesis"

    def draw(self, context):
        lay = self.layout
        c = ctrl()
        if c is None:
            lay.label(text="No se encuentra LRM_CONTROL")
            return

        def sl(col, prop, text):
            col.prop(c, f'["{prop}"]', text=text, slider=True)

        b = lay.box(); b.label(text="Brazo", icon='CON_ROTLIMIT')
        sl(b, "codo", "Codo"); sl(b, "muneca", "Muneca (giro)")
        b = lay.box(); b.label(text="Mano", icon='HAND')
        sl(b, "cierre_mano", "Cierre de mano"); sl(b, "pinza", "Pinza")
        col = b.column(align=True); col.label(text="Dedos")
        sl(col, "indice", "Indice"); sl(col, "medio", "Medio"); sl(col, "anular", "Anular"); sl(col, "menique", "Menique")
        col = b.column(align=True); col.label(text="Pulgar")
        sl(col, "pulgar_flexion", "Flexion"); sl(col, "pulgar_oposicion", "Oposicion")
        b = lay.box(); b.label(text="Vistas", icon='MOD_EXPLODE')
        sl(b, "explosion", "Explosion"); sl(b, "panel_solar", "Panel solar fuera")
        b = lay.box(); b.label(text="Poses rapidas", icon='ARMATURE_DATA')
        g = b.grid_flow(columns=2, align=True)
        for k, (label, _) in POSES.items():
            g.operator("lrm.pose", text=label).pose = k
        b = lay.box(); b.label(text="Animacion", icon='PLAY')
        b.operator("lrm.keyframe", icon='KEY_HLT')
        b.operator("lrm.demo", icon='SEQUENCE')
        b.operator("lrm.clear_anim", icon='X')


CLASSES = (LRM_OT_pose, LRM_OT_keyframe, LRM_OT_clear, LRM_OT_demo, LRM_PT_panel)


def register():
    for cl in CLASSES:
        try:
            bpy.utils.register_class(cl)
        except ValueError:
            bpy.utils.unregister_class(cl)
            bpy.utils.register_class(cl)


register()
