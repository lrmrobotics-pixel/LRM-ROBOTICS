# -*- coding: utf-8 -*-
"""Prueba de la protesis LRM V1 en PyBullet.
    pip install pybullet
    python scripts/pybullet_prueba.py            # ventana 3D con deslizadores por junta
    python scripts/pybullet_prueba.py --demo     # demo: flexion de codo + agarre cilindrico + pinza
    python scripts/pybullet_prueba.py --headless # chequeo sin ventana (carga, limites, autocolision)
"""
import os, sys, math, time
import pybullet as p
import pybullet_data

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "exports", "urdf", "lrm_protesis_v1.urdf")
headless = "--headless" in sys.argv
demo = "--demo" in sys.argv

p.connect(p.DIRECT if headless else p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf", [0, 0, -0.35])
# base fija = socket (en la realidad la sostiene el munon + arnes)
# sin autocolision: PyBullet usa la envolvente convexa de cada malla, y en carcasas huecas eso
# genera contactos falsos. Las interferencias reales ya se verificaron en Blender (lrm_verificar.py).
robot = p.loadURDF(URDF, [0, 0, 0], useFixedBase=True, flags=p.URDF_USE_INERTIA_FROM_FILE)

J = {}
for i in range(p.getNumJoints(robot)):
    info = p.getJointInfo(robot, i)
    J[info[1].decode()] = dict(idx=i, lo=info[8], hi=info[9], eff=info[10], vel=info[11])
print(f"[PB] juntas cargadas: {len(J)}")
for n, j in J.items():
    print(f"   {n:30s} [{math.degrees(j['lo']):6.1f}, {math.degrees(j['hi']):6.1f}] deg  esfuerzo {j['eff']} N*m")
masa = sum(p.getDynamicsInfo(robot, i)[0] for i in range(0, p.getNumJoints(robot)))
print(f"[PB] masa movil (antebrazo+mano) {masa:.3f} kg; el socket es base fija (sin electronica)")

FINGERS = ["index", "middle", "ring", "pinky"]


def pose(q, steps=240):
    """mueve con control de posicion (respeta esfuerzo y velocidad del URDF)"""
    for n, a in q.items():
        j = J[n]
        p.setJointMotorControl2(robot, j["idx"], p.POSITION_CONTROL, targetPosition=a,
                                force=j["eff"], maxVelocity=j["vel"])
    for _ in range(steps):
        p.stepSimulation()
        if not headless:
            time.sleep(1 / 240)


def agarre(cierre, pinza=False):
    q = {}
    for f in FINGERS:
        k = cierre if (not pinza or f == "index") else 0.15
        q.update({f"{f}_mcp_joint": k * math.radians(80), f"{f}_pip_joint": k * math.radians(90),
                  f"{f}_dip_joint": k * math.radians(60)})
    q.update({"thumb_cmc_opposition_joint": cierre * math.radians(40), "thumb_cmc_flexion_joint": cierre * 0.3,
              "thumb_mcp_joint": cierre * math.radians(35), "thumb_ip_joint": cierre * math.radians(45)})
    return q


if headless:
    # 1) cada junta llega a sus limites; 2) contactos de autocolision en los extremos
    for n, j in J.items():
        for tgt in (j["lo"], j["hi"]):
            p.resetJointState(robot, j["idx"], tgt)
    p.performCollisionDetection()
    for n, j in J.items():
        p.resetJointState(robot, j["idx"], 0)
    print("[PB] carga y limites OK")
    pose({"elbow_joint": math.radians(135)}, 600)
    print(f"[PB] codo alcanzo {math.degrees(p.getJointState(robot, J['elbow_joint']['idx'])[0]):.1f} deg "
          "contra gravedad")
    p.disconnect()
    sys.exit(0)

if demo:
    p.resetDebugVisualizerCamera(0.9, 35, -25, [0.38, 0, 0])
    while p.isConnected():
        pose({"elbow_joint": math.radians(90)}, 360)
        pose(agarre(1.0), 300)                      # agarre cilindrico
        pose(agarre(0.0), 200)
        pose(agarre(1.0, pinza=True), 300)          # pinza pulgar-indice
        pose({**agarre(0.0), "wrist_joint": math.radians(80)}, 300)
        pose({"wrist_joint": 0.0, "elbow_joint": 0.0}, 360)
else:
    p.resetDebugVisualizerCamera(0.9, 35, -25, [0.38, 0, 0])
    sl = {n: p.addUserDebugParameter(n, j["lo"], j["hi"], 0.0) for n, j in J.items()}
    while p.isConnected():
        pose({n: p.readUserDebugParameter(s) for n, s in sl.items()}, 4)
