#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial, time, numpy as np, sys, math

# =========================
# CONFIGURACIÓN HARDWARE
# =========================
PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SERVO_IDS = [1, 2, 3, 4, 5, 6]
MOVE_TIME_MS = 1000

CALIB = {
    1: {"min_pos": 992,  "max_pos": 3691, "min_ang": -125, "max_ang": 125,  "sentido": +1},
    2: {"min_pos": 1208, "max_pos": 3822, "min_ang": -35,  "max_ang": 200,  "sentido": -1},
    3: {"min_pos": 152,  "max_pos": 2403, "min_ang": -160, "max_ang": 20,   "sentido": -1},
    4: {"min_pos": 1103, "max_pos": 3438, "min_ang": -90,  "max_ang": 90,   "sentido": -1},
    5: {"min_pos": 196,  "max_pos": 3920, "min_ang": -150, "max_ang": 150,  "sentido": -1},
    6: {"min_pos": 2055, "max_pos": 3400, "min_ang": -100, "max_ang": 20,   "sentido": +1},
}

ADDR_GOAL_POS = 0x2A
INST_WRITE    = 0x03


# =========================
# FUNCIONES DE CONTROL SERIAL
# =========================
def checksum_sum(s): 
    return (~(s & 0xFF)) & 0xFF

def packet(sid, inst, params):
    body = [sid, len(params)+2, inst] + params
    return bytes([0xFF, 0xFF] + body + [checksum_sum(sum(body))])

def limit_angle(sid, angle):
    """Restringe el ángulo a su rango permitido según CALIB."""
    a_min = CALIB[sid]["min_ang"]
    a_max = CALIB[sid]["max_ang"]
    return max(min(angle, max(a_min, a_max)), min(a_min, a_max))

def angle_to_position(sid, angle_deg):
    cal = CALIB[sid]
    a_min, a_max = cal["min_ang"], cal["max_ang"]
    p_min, p_max = cal["min_pos"], cal["max_pos"]
    sentido = cal["sentido"]

    # Limitar ángulo al rango permitido
    angle_deg = limit_angle(sid, angle_deg)

    if abs(a_max - a_min) < 1e-6:
        return int(round((p_min + p_max) / 2))

    # Mapeo lineal según sentido
    if sentido > 0:
        pos = p_min + (p_max - p_min) * (angle_deg - a_min) / (a_max - a_min)
    else:
        pos = p_max - (p_max - p_min) * (angle_deg - a_min) / (a_max - a_min)
    return int(round(pos))

def move_to(ser, sid, pos, time_ms, speed=0):
    pos = max(0, min(4095, int(pos)))
    data = [
        pos & 0xFF, (pos >> 8) & 0xFF,
        time_ms & 0xFF, (time_ms >> 8) & 0xFF,
        speed & 0xFF, (speed >> 8) & 0xFF
    ]
    ser.write(packet(sid, INST_WRITE, [ADDR_GOAL_POS] + data))
    ser.flush()
    time.sleep(0.002)


# =========================
# CINEMÁTICA DIRECTA (DH)
# =========================
L0, L1, L2, L3, L45 = 54.2, 30.5, 116.1, 134.5, 160.0  # mm

def dh_table(theta1, theta2, theta3, theta4, theta5):
    return np.array([
        [theta1 + 0.0,  L0,  L1,   np.pi/2],
        [theta2 + 0.0,  0,   L2,   0.0],
        [theta3 + 0.0,  0,   L3,   0.0],
        [theta4 - np.pi/2, 0,   0,  -np.pi/2],
        [theta5 + 0.0,  L45, 0,   0.0],
    ], dtype=float)

def T_dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct ],
        [ st,  ct*ca, -ct*sa, a*st ],
        [  0,     sa,     ca,    d ],
        [  0,      0,      0,    1 ]
    ])

def fk_5dof(theta1, theta2, theta3, theta4, theta5):
    dh = dh_table(theta1, theta2, theta3, theta4, theta5)
    T = np.eye(4)
    for i in range(5):
        T = T @ T_dh(*dh[i])
    p = T[:3, 3]
    return T, p


# =========================
# CINEMÁTICA INVERSA NUMÉRICA (DLS + sesgos direccionales + acoplamiento 2↔4)
# =========================
def ik_5dof_pos(target_pos, q_init=None, max_iter=200, tol=1e-3):
    if q_init is None:
        q = np.deg2rad([0, 180, -150, 0, 0])
    else:
        q = np.array(q_init, dtype=float)

    lam = 0.01
    bias_gain = np.deg2rad(3.0)   # 🔥 preferencia FUERTE (~3° por iteración)
    k_couple  = 0.02              # 🔥 acoplamiento fuerte servo2→servo4
    q2_ref = np.deg2rad(0)

    for _ in range(max_iter):
        _, p = fk_5dof(*q)
        e = target_pos - p
        if np.linalg.norm(e) < tol:
            return np.rad2deg(q)

        # --- Jacobiano numérico (posición) ---
        J = np.zeros((3, 5))
        h = 1e-5
        for i in range(5):
            q_pert = q.copy()
            q_pert[i] += h
            _, p_pert = fk_5dof(*q_pert)
            J[:, i] = (p_pert - p) / h

        JT = J.T
        dq = JT @ np.linalg.inv(J @ JT + lam**2 * np.eye(3)) @ e

        # ====== SESGOS FUERTES ======
        q_deg = np.rad2deg(q)

        # Servo 2 → preferir > 70°
        if q_deg[1] < 70:
            dq[1] += bias_gain * (1.0 - q_deg[1] / 70.0)  # más empuje cuanto menor sea

        # Servo 3 → preferir < -45°
        if q_deg[2] > -45:
            dq[2] -= bias_gain * (1.0 + q_deg[2] / 45.0)

        # Servo 4 → preferir < 0°
        if q_deg[3] > 0:
            dq[3] -= bias_gain * (1.0 + q_deg[3] / 90.0)

        # ====== ACOPLAMIENTO FUERTE 2 ↔ 4 ======
        # Si servo 2 sube, servo 4 tiende a subir también
        dq[3] += k_couple * (q[1] - q2_ref)

        # ====== Actualización y límites ======
        q += dq * 0.7  # suavizado: evita oscilaciones fuertes
        for i in range(5):
            q[i] = np.clip(q[i],
                np.deg2rad(CALIB[i+1]["min_ang"]),
                np.deg2rad(CALIB[i+1]["max_ang"])
            )

    return np.rad2deg(q)




# =========================
# CONTROL PRINCIPAL
# =========================
def move_arm_to_target(target_pos):
    """Calcula IK y mueve el brazo a una posición cartesiana."""
    q_sol = ik_5dof_pos(target_pos)
    print(f"[IK] Solución (deg): {np.round(q_sol, 3)}")

    with serial.Serial(PORT, BAUD, timeout=0.002) as ser:
        for sid, ang in zip(SERVO_IDS[:5], q_sol):
            pos = angle_to_position(sid, ang)
            move_to(ser, sid, pos, MOVE_TIME_MS)
            print(f"  Servo {sid}: {ang:+6.1f}° → Pos {pos}")
        print("[INFO] Movimiento enviado ✅")


# =========================
# TELEOPERACIÓN SIMPLE (desde consola)
# =========================
def main():
    print("=== TELEOPERACIÓN PIPER 5DOF ===")
    print("Comandos: ")
    print("  w/s → subir/bajar Z")
    print("  a/d → mover Y")
    print("  q/e → mover X")
    print("  o   → abrir gripper")
    print("  c   → cerrar gripper")
    print("  x   → salir")

    pos = np.array([0.0, 325.0, 170.0])  # posición inicial (mm)
    grip = 0  # 0 cerrado, 100 abierto

    while True:
        print(f"\nPosición actual objetivo [mm]: {np.round(pos,2)}")
        cmd = input("→ comando: ").strip().lower()
        if cmd == "x":
            break
        elif cmd == "w": pos[2] += 10
        elif cmd == "s": pos[2] -= 10
        elif cmd == "a": pos[1] -= 10
        elif cmd == "d": pos[1] += 10
        elif cmd == "q": pos[0] -= 10
        elif cmd == "e": pos[0] += 10
        elif cmd == "o": grip = min(grip + 10, 100)  # abrir
        elif cmd == "c": grip = max(grip - 10, 0)    # cerrar
        else:
            print("Comando no reconocido.")
            continue

        # Mover brazo
        move_arm_to_target(pos)

        # Control gripper (servo 6)
        with serial.Serial(PORT, BAUD, timeout=0.002) as ser:
            pos6 = angle_to_position(6, -100 + grip * 1.2)
            move_to(ser, 6, pos6, 500)
            print(f"[GRIPPER] ángulo={-100 + grip * 1.2:.1f}° → pos={pos6}")

if __name__ == "__main__":
    main()