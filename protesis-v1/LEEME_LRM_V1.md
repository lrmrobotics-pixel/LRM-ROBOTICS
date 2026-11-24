# LRM ROBOTICS — Prótesis transhumeral V1 (solo estructura)

Modelo paramétrico generado por script. **No editar el .blend a mano para cambios de diseño**: se cambian
los parámetros en `scripts/lrm_protesis_v1.py` y se regenera (≈15 s).

## Cómo regenerar / verificar / renderizar / exportar
```
blender -b --python scripts/lrm_protesis_v1.py                       # genera LRM_Protesis_V1.blend
blender -b LRM_Protesis_V1.blend --python scripts/lrm_verificar.py   # interferencias + barrido de juntas
blender -b LRM_Protesis_V1.blend --python scripts/lrm_render.py -- all          # 4 vistas finales
blender -b LRM_Protesis_V1.blend --python scripts/lrm_render.py -- all preview  # vista previa rápida
blender -b LRM_Protesis_V1.blend --python scripts/lrm_exportar.py   # STL por pieza (mm) + GLB con juntas
```
## Mover la prótesis (panel de control)
En la vista 3D pulsa **N** → pestaña **"LRM Protesis"**:
- **Brazo**: Codo (0–135°), Muñeca (giro ±90°).
- **Mano**: Cierre de mano (agarre cilíndrico), Pinza, cada dedo por separado, Pulgar (flexión y oposición).
- **Vistas**: Explosión (0 = ensamblada, 1 = explosionada), Panel solar fuera.
- **Poses rápidas**: Reposo, Agarre cilíndrico, Pinza, Señalar, Pulgar arriba, Saludo, Sostener vaso, Vista explosionada, Retirar panel solar.
- **Animación**: *Crear animación demo* (frames 1–250, luego Espacio), *Guardar pose en este frame*
  (para armar tu propia animación: cambia de frame, mueve los deslizadores y guarda) y *Borrar animación*.

El escenario (piso, luces, cámaras, aros) está oculto en la vista de trabajo pero sí sale en los renders;
para verlo, pulsa **Alt+H** en la vista 3D o el ojo de la colección `ESCENARIO_LRM` en el Outliner.

Si el panel no aparece, Blender bloqueó los scripts del archivo: pulsa **"Permitir ejecución"** en el aviso
amarillo al abrir, o activa *Preferencias → Guardar y cargar → Ejecutar scripts de Python automáticamente*.
Aunque esté bloqueado, los deslizadores siguen funcionando: selecciona **LRM_CONTROL** en el Outliner →
*Propiedades de objeto → Propiedades personalizadas*.

## Convenciones
- Unidades: metros internos, la escena muestra mm; STL exportados en mm.
- Z arriba. Brazo **derecho** extendido hacia +X, palma hacia −Z (pronada), pulgar hacia +Y.
- Origen = cara proximal del socket. Longitud socket → punta del dedo medio ≈ 732 mm.
- Cada pieza física es un objeto independiente con propiedades `material`, `physics` (rigid/flexible),
  `kin_link`, `size_mm`, `descripcion`.
- Juntas = Empties en `00_CINEMATICA_juntas`, pivote exacto en el eje, rotación local 0 en reposo,
  restricción *Limit Rotation* y propiedades `axis_local`, `limit_deg`, `rest_deg`.
  Todo queda también en `exports/LRM_V1_cinematica.json` (para generar el URDF).

| Junta | Tipo | Rango |
|---|---|---|
| elbow_joint | revoluta, eje medio-lateral | 0–135° |
| wrist_joint | revoluta, eje largo antebrazo | ±90° |
| *_mcp / *_pip / *_dip (4 dedos) | revoluta | 0–90° |
| thumb_cmc_opposition_joint + thumb_cmc_flexion_joint | 2 GDL | 0–45° / 0–30° |
| thumb_mcp_joint / thumb_ip_joint | revoluta | 0–60° / 0–90° |

Verificación actual: **0 interferencias** en reposo; codo 0–135°, muñeca ±90° y todas las falanges 0–90° sin choques.

## Decisiones de diseño (y por qué)
1. **Nivel transhumeral completo**: el brief mezclaba socket transhumeral y transradial; una prótesis real usa uno.
   El antebrazo ya no tiene socket: es estructura hueca para electrónica. El liner pasó al socket del brazo.
2. **Liner de silicona médica curada por platino (ISO 10993-5)** en todo contacto con piel (informe, sec. 6.3);
   el rPET queda solo como estructura.
3. **Electrodos EMG dentro del socket** (bíceps y tríceps), atravesando socket y liner hasta la piel: en un
   transhumeral el músculo residual está en el brazo, no en el antebrazo.
4. **Almacenamiento dual** (informe, sec. 7): pack 2×18650 en el brazo + compartimento LiPo/supercapacitor con
   tapa sin herramientas en el antebrazo.
5. **Panel solar ≈ 80 cm²** (dentro de los 80–100 cm² del informe), pieza independiente con clip inferior,
   muesca para levantarlo con el dedo y conector de 4 pads dorados sobre la bandeja rebajada.
6. **Codo**: pivote 16 mm hacia anterior y chaflanes de holgura (como los codos protésicos comerciales) para
   llegar a 135° sin choque; servo 40×20 mm en el húmero, piñón z24 → corona z40 en el antebrazo;
   tope mecánico 0–135°; tornillos de fricción coaxiales al pasador.
7. **Pulgar anatómico**: el pulgar real tiene metacarpiano + 2 falanges (no 3): `thumb_metacarpal`,
   `thumb_phalanx_proximal`, `thumb_phalanx_distal`, más la junta IP.
8. **Dedos**: chaflanes palmares a 45° en cada falange → flexión de 90° sin choque; tendón de nylon por canal
   palmar (brazo de momento real de flexor) con camisa PTFE; resorte de torsión de retorno en cada MCP.
9. **Tendones virtuales** (`08_TENDONES_virtuales`): curvas con hooks a cada eslabón, siguen el movimiento;
   no se renderizan.
10. **Escala**: joven/mujer adulta p50 (perfil del informe), ≈ 70 cm hombro–punta; el 55–60 cm del brief
    corresponde a un niño de ~10 años.

## URDF / simulación
```
blender -b LRM_Protesis_V1.blend --python scripts/lrm_urdf.py          # exports/urdf/lrm_protesis_v1.urdf + meshes/
blender -b LRM_Protesis_V1.blend --python scripts/lrm_urdf_validar.py  # FK URDF vs Blender (200 poses), inercias
C:\Users\yulia\miniforge3\envs\lrm\python.exe scripts/pybullet_prueba.py --demo       # demo 3D
C:\Users\yulia\miniforge3\envs\lrm\python.exe scripts/pybullet_prueba.py             # deslizadores por junta
C:\Users\yulia\miniforge3\envs\lrm\python.exe scripts/pybullet_prueba.py --headless  # chequeo sin ventana
```
En Windows PyBullet no se instala con pip (PyPI no tiene binarios); se usa el entorno conda `lrm` (Miniforge).
- 19 eslabones, 18 juntas revolute, base fija = socket. Masa estructural ≈ 0,89 kg (sin electrónica).
- Masas e inercias integradas de las mallas reales con densidad por material y relleno FDM.
- Cero de las juntas = dedos rectos (en Blender los dedos tienen una flexión natural de reposo).
- Colisión = misma malla; PyBullet la convierte en envolvente convexa (para colisión fina: VHACD en V2).

## Pendiente para V2
- Probar en Gazebo; descomposición convexa (VHACD) para colisiones finas.
- Rig con armature (además de la jerarquía de Empties) para animación en Unity.
- Escaneo del muñón real para el socket personalizado.
