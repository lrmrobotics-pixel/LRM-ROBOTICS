# LRM ROBOTICS

Innovación social que nació en un colegio: prótesis biónicas mioeléctricas de bajo costo, impresas en 3D con plástico reciclado (rPET) y con carga solar.

## Contenido

- [`protesis-v1/`](protesis-v1/): prótesis transhumeral V1 (estructura).
  - Modelo paramétrico de Blender generado por script (`scripts/lrm_protesis_v1.py`).
  - STL por pieza en mm, listos para imprimir (`exports/stl_por_pieza_mm/`).
  - URDF y cinemática para simulación (`exports/urdf/`, PyBullet).
  - Renders y poses (`renders/`).
  - Instrucciones detalladas: [`protesis-v1/LEEME_LRM_V1.md`](protesis-v1/LEEME_LRM_V1.md).

## Contribuir

Issues y pull requests son bienvenidos: mejoras de diseño, electrónica EMG (ESP32), firmware, documentación.

## Licencia

[MIT](LICENSE).
