#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import subprocess, serial, time, numpy as np

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

ADDR_GOAL_POS = 0x2A
INST_WRITE    = 0x03

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

# =========================
# FUNCIÓN PARA EJECUTAR IK C++
# =========================
def run_cpp_ik():
    """Ejecuta el binario C++ lero_ik y devuelve los 5 ángulos [°]."""
    try:
        # Llamar al programa y capturar la salida
        output = subprocess.check_output(["./lero_ik"], text=True)
        for line in output.splitlines():
            if line.startswith("IK_RESULT"):
                parts = line.split()[1:]
                return [float(p) for p in parts]
        raise RuntimeError("No se encontró línea IK_RESULT en la salida")
    except subprocess.CalledProcessError as e:
        print("[ERROR] Falló la ejecución de lero_ik:", e)
        return None

# =========================
# PROGRAMA PRINCIPAL
# =========================
def main():
    print("[INFO] Ejecutando solver IK en C++ ...")
    angles_deg = run_cpp_ik()
    if not angles_deg:
        print("[ERROR] No se pudieron obtener ángulos del IK.")
        return

    print(f"[OK] IK retornó ángulos: {np.round(angles_deg,3)}°")

    # Limitar y mover
    print(f"[INFO] Conectando a {PORT} @ {BAUD} ...")
    with serial.Serial(PORT, BAUD, timeout=0.002) as ser:
        for sid, ang in zip(SERVO_IDS[:5], angles_deg):
            ang = max(min(ang, 150), -150)
            pos = angle_to_position(sid, ang)
            move_to(ser, sid, pos, 1000)
            print(f"  Servo {sid}: {ang:+.1f}° → pos {pos}")
        print("[INFO] Movimiento enviado ✅")

if __name__ == "__main__":
    main()
