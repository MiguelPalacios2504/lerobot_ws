import time
import serial

# ====== CONFIG ======
PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SERVO_ID = 6

POS_A = 3223                # ~180°
POS_B = 2559                # ~90°
MOVE_TIME_MS = 800  # tiempo de movimiento (ms)
PAUSA_MS = 300      # pausa entre movimientos (ms)
# ====================

# Registros / instrucciones protocolo SC/STS
ADDR_GOAL_POS   = 0x2A  # 2 bytes (L,H)
ADDR_MOVE_TIME  = 0x2C  # 2 bytes (L,H)
ADDR_MOVE_SPEED = 0x2E  # 2 bytes (L,H) (si tu modelo no lo usa, déjalo 0)
INST_WRITE = 0x03

def checksum_sum(s): return (~(s & 0xFF)) & 0xFF

def packet(sid, inst, params):
    length = len(params) + 2
    body = [sid, length, inst] + params
    return bytes([0xFF, 0xFF] + body + [checksum_sum(sum(body))])

def write_regs(ser, sid, start_addr, data_bytes):
    ser.write(packet(sid, INST_WRITE, [start_addr] + data_bytes))
    ser.flush()

def move_to(ser, sid, pos, time_ms, speed=0):
    pos = max(0, min(4095, int(pos)))
    time_ms = max(0, min(0xFFFF, int(time_ms)))
    speed = max(0, min(0x03FF, int(speed)))
    data = [
        pos & 0xFF, (pos >> 8) & 0xFF,
        time_ms & 0xFF, (time_ms >> 8) & 0xFF,
        speed & 0xFF, (speed >> 8) & 0xFF
    ]
    write_regs(ser, sid, ADDR_GOAL_POS, data)

def main():
    print(f"[INFO] Abriendo {PORT} @ {BAUD} (ID={SERVO_ID})")
    with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
        target = POS_A
        print("[INFO] A ↔ B en bucle. Ctrl+C para salir.")
        while True:
            print(f"-> {target}")
            move_to(ser, SERVO_ID, target, MOVE_TIME_MS, speed=0)
            time.sleep(MOVE_TIME_MS/1000 + PAUSA_MS/1000)
            target = POS_B if target == POS_A else POS_A

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Salido por usuario.")