#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial, time, sys, struct, numpy as np

# =========================
# CONFIGURACIÓN HARDWARE
# =========================
PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SERVO_IDS = [1, 2, 3, 4, 5, 6]

CALIB = {
    1: {"min_pos": 992,  "max_pos": 3691, "min_ang": -125, "max_ang": 125,  "sentido": +1},
    2: {"min_pos": 1208, "max_pos": 3822, "min_ang": -35,  "max_ang": 200,  "sentido": -1},
    3: {"min_pos": 152,  "max_pos": 2403, "min_ang": -180, "max_ang": 15,   "sentido": -1},
    4: {"min_pos": 1103, "max_pos": 3438, "min_ang": -90,  "max_ang": 90,   "sentido": -1},
    5: {"min_pos": 196,  "max_pos": 3920, "min_ang": -150, "max_ang": 150,  "sentido": -1},
    6: {"min_pos": 2055, "max_pos": 3400, "min_ang": -100, "max_ang": 20,   "sentido": +1},
}

MOVE_TIME_MS = 1000
PAUSA_MS     = 500

ADDR_GOAL_POS   = 0x2A
ADDR_START_READ = 0x38
READ_LENGTH     = 6
INST_WRITE      = 0x03
INST_SYNC_READ  = 0x82
# =========================


# =========================
# CINEMÁTICA DIRECTA (DH)
# =========================
L0  = 54.2
L1  = 30.5
L2  = 116.1
L3  = 134.5
L45 = 160.0  # L4 + L5

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
    """Devuelve T05 (4x4) y posición del gripper (x,y,z) [mm]."""
    dh = dh_table(theta1, theta2, theta3, theta4, theta5)
    T = np.eye(4)
    for i in range(5):
        T = T @ T_dh(*dh[i])
    p = T[:3, 3]
    return T, p
# =========================


# =========================
# FUNCIONES SERIALES
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

def move_all_by_angle(ser, angles_deg):
    print("\n[INFO] Moviendo servos por ángulos:")
    for sid, ang in zip(SERVO_IDS, angles_deg):
        # Restringir automáticamente
        ang_limited = limit_angle(sid, ang)
        if ang != ang_limited:
            print(f"  ⚠️ Servo {sid}: {ang:+.1f}° fuera de rango → limitado a {ang_limited:+.1f}°")
            ang = ang_limited

        pos = angle_to_position(sid, ang)
        move_to(ser, sid, pos, MOVE_TIME_MS)
        print(f"  Servo {sid}: {ang:+6.1f}° → Pos {pos}")
    print("[INFO] Movimiento enviado ✅")
# =========================


# =========================
# PROGRAMA PRINCIPAL
# =========================
def main():
    # Ejemplo: ángulos de destino [°]
    target_angles_deg = [0, 90, -90, 0, 0, 0]  # algunos fuera de rango a propósito

    print(f"[INFO] Conectando a {PORT} @ {BAUD}...")
    with serial.Serial(PORT, BAUD, timeout=0.002) as ser:
        move_all_by_angle(ser, target_angles_deg)

    # ---- Cálculo de Cinemática Directa ----
    q_rad = np.deg2rad([limit_angle(i, ang) for i, ang in zip(SERVO_IDS[:5], target_angles_deg[:5])])
    T05, pos = fk_5dof(*q_rad)

    print("\n[RESULTADOS FK]")
    print("T_0_5 =\n", np.round(T05, 4))
    print("Posición del gripper [mm]:", np.round(pos, 3))
    print("Dirección del gripper (eje Z):", np.round(T05[:3, 2], 3))


if __name__ == "__main__":
    main()
