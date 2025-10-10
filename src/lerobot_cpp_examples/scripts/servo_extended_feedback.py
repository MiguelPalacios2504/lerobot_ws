#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial, time, sys, struct

# =========================
# CONFIGURACIÓN
# =========================
PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SERVO_IDS = [1, 2, 3, 4, 5, 6]
SERVO_ID_MOVE = 6

ADDR_GOAL_POS     = 0x2A
ADDR_START_READ   = 0x38           # posición actual
READ_LENGTH       = 6              # leer solo pos, vel, curr
INST_WRITE        = 0x03
INST_SYNC_READ    = 0x82

POS_A = 3223
POS_B = 2559
MOVE_TIME_MS = 800
PAUSA_MS = 300
# =========================


def checksum_sum(s): 
    return (~(s & 0xFF)) & 0xFF


def packet(sid, inst, params):
    length = len(params) + 2
    body = [sid, length, inst] + params
    return bytes([0xFF, 0xFF] + body + [checksum_sum(sum(body))])


def move_to(ser, sid, pos, time_ms, speed=0):
    """Envía comando de movimiento."""
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


def sync_read_fast(ser, ids):
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.0008)
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



def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.002)
    print(f"[INFO] Conectado a {PORT} @ {BAUD}")
    print(f"[INFO] Movimiento servo {SERVO_ID_MOVE} entre {POS_A} ↔ {POS_B}")

    try:
        target = POS_A
        while True:
            move_to(ser, SERVO_ID_MOVE, target, MOVE_TIME_MS)
            t_start = time.time()
            period = 0.006  # ~166 Hz
            while (time.time() - t_start) < (MOVE_TIME_MS/1000 + PAUSA_MS/1000):
                fb = sync_read_fast(ser, SERVO_IDS)
                out = []
                for sid in SERVO_IDS:
                    if sid in fb:
                        d = fb[sid]
                        out.append(
                            f"ID{sid}:P{d['pos']:4d} "
                            f"S{d['speed']:4d} "
                            f"I{d['curr']:5d}"
                        )
                    else:
                        out.append(f"ID{sid}: ----")
                line = " | ".join(out)[:180]
                sys.stdout.write(f"\r{line:<180}")  # imprime en una sola línea, relleno para borrar restos
                sys.stdout.flush()
                time.sleep(period)
            target = POS_B if target == POS_A else POS_A
    except KeyboardInterrupt:
        sys.stdout.write("\r")         # 🔹 limpia la línea actual
        sys.stdout.flush()
        print("\n[INFO] Salida por usuario.")
    finally:
        ser.close()
        print("[INFO] Puerto cerrado correctamente.")


if __name__ == "__main__":
    main()
