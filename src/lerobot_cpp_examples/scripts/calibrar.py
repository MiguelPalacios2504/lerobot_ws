#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial, time

# =========================
# CONFIGURACIÓN
# =========================
PORT = "/dev/ttyACM0"
BAUD = 1_000_000
SERVO_IDS = [1, 2, 3, 4, 5, 6]
ADDR_MODE = 0x21
INST_WRITE = 0x03
INST_READ  = 0x02
# =========================


def checksum_sum(s):
    return (~(s & 0xFF)) & 0xFF


def make_packet(sid, inst, params):
    body = [sid, len(params) + 2, inst] + params
    chk = checksum_sum(sum(body))
    return bytes([0xFF, 0xFF] + body + [chk])


def read_mode(ser, sid):
    """Lee el modo actual (0=servo, 1=multiturno)"""
    pkt = make_packet(sid, INST_READ, [ADDR_MODE, 1])
    ser.write(pkt)
    ser.flush()
    resp = ser.read(8)
    if len(resp) >= 7 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == sid:
        return resp[5]
    return None


def write_mode(ser, sid, mode):
    """Escribe el modo del servo"""
    pkt = make_packet(sid, INST_WRITE, [ADDR_MODE, mode])
    ser.write(pkt)
    ser.flush()
    time.sleep(0.02)


def main():
    print(f"[INFO] Conectando a {PORT} @ {BAUD}")
    try:
        with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
            for sid in SERVO_IDS:
                print(f"\n→ Servo {sid}:")
                current_mode = read_mode(ser, sid)
                if current_mode is None:
                    print("   ⚠️  No responde")
                    continue

                if current_mode == 0:
                    print("   ✅ Ya está en modo 0 (servo limitado)")
                elif current_mode == 1:
                    print("   🔁 Modo multiturno detectado → cambiando a 0...")
                    write_mode(ser, sid, 0)
                    time.sleep(0.05)
                    new_mode = read_mode(ser, sid)
                    if new_mode == 0:
                        print("   ✅ Modo cambiado correctamente")
                    else:
                        print("   ❌ No se pudo cambiar el modo")
                else:
                    print(f"   ❓ Valor desconocido leído: {current_mode}")

        print("\n[OK] Verificación completa. Todos los servos revisados.")
        print("[INFO] Ahora puedes ejecutar tu calibración:")
        print("       → python3 calibrar_rangos.py")

    except serial.SerialException:
        print(f"[ERROR] No se puede abrir el puerto {PORT}.")
    except Exception as e:
        print(f"[ERROR] {e}")


if __name__ == "__main__":
    main()
