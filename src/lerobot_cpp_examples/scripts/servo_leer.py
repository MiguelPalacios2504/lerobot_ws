#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial, time, sys, struct, threading

# =========================
# CONFIGURACIÓN
# =========================
PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SERVO_IDS = [1, 2, 3, 4, 5, 6]

CALIB = {
    1: {"min_pos": 992,  "max_pos": 3691, "min_ang": -125, "max_ang": 125,  "sentido": +1},
    2: {"min_pos": 1208, "max_pos": 3822, "min_ang": -40,  "max_ang": 200,  "sentido": -1},
    3: {"min_pos": 152,  "max_pos": 2403, "min_ang": -160, "max_ang": 20,   "sentido": -1},
    4: {"min_pos": 1103, "max_pos": 3438, "min_ang": -100, "max_ang": 100,  "sentido": -1},
    5: {"min_pos": 196,  "max_pos": 3920, "min_ang": 150,  "max_ang": -150, "sentido": -1},
    6: {"min_pos": 2055, "max_pos": 3400, "min_ang": 20,   "max_ang": -100, "sentido": +1},
}

MOVE_TIME_MS = 1500
PAUSA_MS     = 800

ADDR_GOAL_POS   = 0x2A
ADDR_START_READ = 0x38
READ_LENGTH     = 6
INST_WRITE      = 0x03
INST_SYNC_READ  = 0x82
# =========================


# === FUNCIONES DE COMUNICACIÓN ===
def checksum_sum(s): 
    return (~(s & 0xFF)) & 0xFF

def packet(sid, inst, params):
    body = [sid, len(params)+2, inst] + params
    return bytes([0xFF, 0xFF] + body + [checksum_sum(sum(body))])

def sync_read_fast(ser, ids):
    """Lee posiciones de múltiples servos."""
    ser.write(packet(0xFE, INST_SYNC_READ, [ADDR_START_READ, READ_LENGTH, len(ids)] + ids))
    ser.flush()

    expected = (READ_LENGTH + 6) * len(ids)
    raw = ser.read(expected * 2)
    data = {}
    i = 0
    while i + READ_LENGTH + 5 < len(raw):
        if raw[i] == 0xFF and raw[i+1] == 0xFF:
            sid = raw[i+2]
            if sid in ids:
                block = raw[i+5:i+5+READ_LENGTH]
                pos = block[0] | (block[1] << 8)
                data[sid] = pos
                i += READ_LENGTH + 6
                continue
        i += 1
    return data


# === CONVERSIÓN POSICIÓN ↔ ÁNGULO ===
def position_to_angle(sid, pos):
    cal = CALIB[sid]
    p_min, p_max = cal["min_pos"], cal["max_pos"]
    a_min, a_max = cal["min_ang"], cal["max_ang"]
    sentido = cal["sentido"]

    pos = max(min(pos, max(p_min, p_max)), min(p_min, p_max))
    if sentido > 0:
        ang = a_min + (a_max - a_min) * (pos - p_min) / (p_max - p_min)
    else:
        ang = a_min + (a_max - a_min) * (p_max - pos) / (p_max - p_min)
    return ang

def angle_to_position(sid, angle_deg):
    cal = CALIB[sid]
    a_min, a_max = cal["min_ang"], cal["max_ang"]
    p_min, p_max = cal["min_pos"], cal["max_pos"]
    sentido = cal["sentido"]

    angle_deg = max(min(angle_deg, max(a_min, a_max)), min(a_min, a_max))
    if sentido > 0:
        pos = p_min + (p_max - p_min) * (angle_deg - a_min) / (a_max - a_min)
    else:
        pos = p_max - (p_max - p_min) * (angle_deg - a_min) / (a_max - a_min)
    return int(round(pos))


# === MOVIMIENTO ===
def move_to(ser, sid, pos, time_ms):
    pos = max(0, min(4095, int(pos)))
    data = [
        pos & 0xFF, (pos >> 8) & 0xFF,
        time_ms & 0xFF, (time_ms >> 8) & 0xFF,
        0, 0
    ]
    ser.write(packet(sid, INST_WRITE, [ADDR_GOAL_POS] + data))
    ser.flush()
    time.sleep(0.002)

def move_all_by_angle(ser, angles_deg):
    for sid, ang in zip(SERVO_IDS, angles_deg):
        pos = angle_to_position(sid, ang)
        move_to(ser, sid, pos, MOVE_TIME_MS)
    ser.flush()


# === MONITOREO EN VIVO ===
def monitor_loop(ser):
    """Lectura continua siempre activa."""
    while True:
        fb = sync_read_fast(ser, SERVO_IDS)
        out = []
        for sid in SERVO_IDS:
            if sid in fb:
                pos = fb[sid]
                ang = position_to_angle(sid, pos)
                out.append(f"ID{sid}:{ang:+7.2f}°")
            else:
                out.append(f"ID{sid}: ----")
        sys.stdout.write("\r" + " | ".join(out) + " " * 10)
        sys.stdout.flush()
        time.sleep(0.05)


# === MAIN ===
def main():
    pose_A = [0, 90, 0, 0, 0, 0]
    pose_B = [0, 90, 0, 0, 45, 0]

    print(f"[INFO] Conectando a {PORT} @ {BAUD}...")
    with serial.Serial(PORT, BAUD, timeout=0.01) as ser:
        # Lanzar hilo de monitoreo continuo
        t = threading.Thread(target=monitor_loop, args=(ser,), daemon=True)
        t.start()

        while True:
            move_all_by_angle(ser, pose_A)
            time.sleep(MOVE_TIME_MS/1000 + PAUSA_MS/1000)
            move_all_by_angle(ser, pose_B)
            time.sleep(MOVE_TIME_MS/1000 + PAUSA_MS/1000)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Finalizado por usuario.")
