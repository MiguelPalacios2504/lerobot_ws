#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial, time, sys, struct, threading, termios, tty

# =========================
# CONFIGURACIÓN
# =========================
PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SERVO_IDS = [1, 2, 3, 4, 5, 6]

# === Calibración ===
CALIB = {
    1: {"min_pos": 992,  "max_pos": 3691, "min_ang": -125, "max_ang": 125,  "sentido": +1},
    2: {"min_pos": 1208, "max_pos": 3822, "min_ang": -40,  "max_ang": 200,  "sentido": -1},
    3: {"min_pos": 152,  "max_pos": 2403, "min_ang": -160, "max_ang": 20,   "sentido": -1},
    4: {"min_pos": 1103, "max_pos": 3438, "min_ang": -100, "max_ang": 100,  "sentido": -1},
    5: {"min_pos": 196,  "max_pos": 3920, "min_ang": -150, "max_ang": 150,  "sentido": -1},
    6: {"min_pos": 2055, "max_pos": 3400, "min_ang": -100, "max_ang": 20,   "sentido": +1},
}

MOVE_TIME_MS = 400
INCREMENT = 3.0  # paso angular por tecla
# =========================


# === Utilidades de comunicación ===
def checksum_sum(s): 
    return (~(s & 0xFF)) & 0xFF

def packet(sid, inst, params):
    body = [sid, len(params)+2, inst] + params
    return bytes([0xFF, 0xFF] + body + [checksum_sum(sum(body))])

ADDR_GOAL_POS   = 0x2A
ADDR_START_READ = 0x38
READ_LENGTH     = 6
INST_WRITE      = 0x03
INST_SYNC_READ  = 0x82


def sync_read_fast(ser, ids):
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


# === Conversión posición ↔ ángulo ===
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


# === Movimiento ===
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


# === Lectura de teclado sin bloqueo ===
def get_key():
    """Lee una tecla sin bloquear el hilo principal."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


# === Monitoreo en vivo ===
def monitor_loop(ser, current_angles):
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
        sys.stdout.write("\r" + " | ".join(out) + " " * 20)
        sys.stdout.flush()
        time.sleep(0.05)


# === MAIN ===
def main():
    print(f"[INFO] Conectando a {PORT} @ {BAUD}...")
    with serial.Serial(PORT, BAUD, timeout=0.01) as ser:
        # Posiciones iniciales
        current_angles = [0, 90, 0, 0, 0, 0]

        # Hilo de monitoreo
        t = threading.Thread(target=monitor_loop, args=(ser, current_angles), daemon=True)
        t.start()

        print("\n[CONTROL TECLADO]")
        print("W/S → Servo1 | E/D → Servo2 | R/F → Servo3 | T/G → Servo4 | Y/H → Servo5 | U/J → Servo6 | Q → salir\n")

        while True:
            key = get_key().lower()

            # Mapeo de teclas a servos
            if key == "w": current_angles[0] += INCREMENT
            elif key == "s": current_angles[0] -= INCREMENT
            elif key == "e": current_angles[1] += INCREMENT
            elif key == "d": current_angles[1] -= INCREMENT
            elif key == "r": current_angles[2] += INCREMENT
            elif key == "f": current_angles[2] -= INCREMENT
            elif key == "t": current_angles[3] += INCREMENT
            elif key == "g": current_angles[3] -= INCREMENT
            elif key == "y": current_angles[4] += INCREMENT
            elif key == "h": current_angles[4] -= INCREMENT
            elif key == "u": current_angles[5] += INCREMENT
            elif key == "j": current_angles[5] -= INCREMENT
            elif key == "q":
                print("\n[INFO] Finalizado por usuario.")
                break

            # Enviar nuevo ángulo
            for sid, ang in zip(SERVO_IDS, current_angles):
                pos = angle_to_position(sid, ang)
                move_to(ser, sid, pos, MOVE_TIME_MS)
            ser.flush()
            time.sleep(0.05)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Finalizado por usuario.")
