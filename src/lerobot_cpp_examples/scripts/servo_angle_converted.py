#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial, time, sys, struct

# =========================
# CONFIGURACIÓN GLOBAL
# =========================
PORT = "/dev/ttyACM0"
BAUD = 1_000_000

SERVO_IDS = [1, 2, 3, 4, 5, 6]

# --- Calibración por servo ---
# min_pos, max_pos = valores de posición (0–4095)
# min_ang, max_ang = límites articulares (°)
# sentido = +1 si al aumentar el ángulo aumenta la posición, -1 si ocurre lo contrario
CALIB = {
    1: {"min_pos": 992,  "max_pos": 3691, "min_ang": -125, "max_ang": 125,  "sentido": +1},
    2: {"min_pos": 1208, "max_pos": 3822, "min_ang": -40,  "max_ang": 200,  "sentido": -1},
    3: {"min_pos": 152,  "max_pos": 2403, "min_ang": -160,   "max_ang": 20, "sentido": -1},
    4: {"min_pos": 1103, "max_pos": 3438, "min_ang": -100,   "max_ang": 100,  "sentido": -1},
    5: {"min_pos": 196,  "max_pos": 3920, "min_ang": 150,  "max_ang": -150, "sentido": -1},
    6: {"min_pos": 2055, "max_pos": 3400, "min_ang": 20,   "max_ang": -100, "sentido": +1},
}

# --- Movimiento ---
MOVE_TIME_MS = 1000
PAUSA_MS     = 500

ADDR_GOAL_POS   = 0x2A
ADDR_START_READ = 0x38
READ_LENGTH     = 6
INST_WRITE      = 0x03
INST_SYNC_READ  = 0x82
# =========================


def checksum_sum(s): 
    return (~(s & 0xFF)) & 0xFF

def packet(sid, inst, params):
    body = [sid, len(params)+2, inst] + params
    return bytes([0xFF, 0xFF] + body + [checksum_sum(sum(body))])


# === CONVERSIÓN ÁNGULO → POSICIÓN ===
def angle_to_position(sid, angle_deg):
    cal = CALIB[sid]
    a_min, a_max = cal["min_ang"], cal["max_ang"]
    p_min, p_max = cal["min_pos"], cal["max_pos"]
    sentido = cal["sentido"]

    if abs(a_max - a_min) < 1e-6:
        return int(round((p_min + p_max) / 2))

    # Limitar ángulo al rango permitido
    angle_deg = max(min(angle_deg, max(a_min, a_max)), min(a_min, a_max))

    # Mapeo lineal según sentido
    if sentido > 0:
        pos = p_min + (p_max - p_min) * (angle_deg - a_min) / (a_max - a_min)
    else:
        pos = p_max - (p_max - p_min) * (angle_deg - a_min) / (a_max - a_min)
    return int(round(pos))


# === LECTURA SYNC ===
def sync_read_fast(ser, ids):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(packet(0xFE, INST_SYNC_READ, [ADDR_START_READ, READ_LENGTH, len(ids)] + ids))
    ser.flush()

    expected = (READ_LENGTH + 6) * len(ids)
    raw = b""
    t0 = time.time()
    while len(raw) < expected and (time.time() - t0) < 0.02:
        chunk = ser.read(expected - len(raw))
        if not chunk:
            break
        raw += chunk

    data = {}
    i = 0
    while i + READ_LENGTH + 5 < len(raw):
        if raw[i] == 0xFF and raw[i+1] == 0xFF:
            sid = raw[i+2]
            if sid in ids:
                block = raw[i+5:i+5+READ_LENGTH]
                if len(block) == READ_LENGTH:
                    pos   = block[0] | (block[1] << 8)
                    speed = struct.unpack("<h", bytes(block[2:4]))[0]
                    curr  = struct.unpack("<h", bytes(block[4:6]))[0]
                    data[sid] = {"pos": pos, "speed": speed, "curr": curr}
                i += READ_LENGTH + 6
                continue
        i += 1
    return data


# === MOVIMIENTO DE TODOS LOS SERVOS ===
def move_all_by_angle(ser, angles_deg):
    for sid, ang in zip(SERVO_IDS, angles_deg):
        pos = angle_to_position(sid, ang)
        move_to(ser, sid, pos, MOVE_TIME_MS)
        print(f"  Servo {sid}: {ang:+6.1f}° → Pos {pos}")
    print("[INFO] Movimiento enviado ✅")


# === MOVIMIENTO INDIVIDUAL ===
def move_to(ser, sid, pos, time_ms, speed=0):
    pos = max(0, min(4095, int(pos)))
    time_ms = max(0, min(0xFFFF, int(time_ms)))
    speed = max(0, min(0x03FF, int(speed)))
    data = [
        pos & 0xFF, (pos >> 8) & 0xFF,
        time_ms & 0xFF, (time_ms >> 8) & 0xFF,
        speed & 0xFF, (speed >> 8) & 0xFF
    ]
    ser.write(packet(sid, INST_WRITE, [ADDR_GOAL_POS] + data))
    ser.flush()
    time.sleep(0.002)


# === MAIN ===
def main():
    # Ejemplo: ángulos de destino en grados
    target_angles = [90, 90, 0, 0, 0, 0]  # puedes cambiarlos libremente

    print(f"[INFO] Conectando a {PORT} @ {BAUD}...")
    with serial.Serial(PORT, BAUD, timeout=0.002) as ser:
        print("[INFO] Moviendo servos por ángulos...")
        move_all_by_angle(ser, target_angles)

        # Mostrar feedback
        print("[INFO] Leyendo feedback...")
        t_start = time.time()
        while (time.time() - t_start) < (MOVE_TIME_MS/1000 + PAUSA_MS/1000):
            fb = sync_read_fast(ser, SERVO_IDS)
            out = []
            for sid in SERVO_IDS:
                if sid in fb:
                    out.append(f"ID{sid}:P{fb[sid]['pos']:4d}")
                else:
                    out.append(f"ID{sid}: ----")
            sys.stdout.write("\r" + " | ".join(out))
            sys.stdout.flush()
            time.sleep(0.01)

    print("\n[OK] Movimiento completo ✅")


if __name__ == "__main__":
    main()
