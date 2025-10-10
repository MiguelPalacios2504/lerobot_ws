import serial
import time
import sys

# =========================
# CONFIGURACIÓN
# =========================
PORT = "/dev/ttyACM0"           # o /dev/ttyUSB0 en Linux
BAUD = 1_000_000
SERVO_IDS = [1, 2, 3, 4, 5, 6]   # Los 6 joints
ADDR_PRESENT_POS = 0x38          # Dirección de posición actual
INST_READ = 0x02                 # Instrucción de lectura
# =========================


def checksum_sum(s):
    """Calcula checksum para protocolo SC/STS"""
    return (~(s & 0xFF)) & 0xFF


def make_read_packet(sid, addr, length):
    """Crea un paquete de lectura"""
    body = [sid, 4, INST_READ, addr, length]
    chk = checksum_sum(sum(body))
    return bytes([0xFF, 0xFF] + body + [chk])


def read_position(ser, sid):
    """Lee posición actual de un servo"""
    pkt = make_read_packet(sid, ADDR_PRESENT_POS, 2)
    ser.write(pkt)
    ser.flush()
    resp = ser.read(8)  # respuesta esperada
    if len(resp) < 8 or resp[0] != 0xFF or resp[1] != 0xFF:
        return None
    pos = resp[5] | (resp[6] << 8)
    return pos


def main():
    print(f"[INFO] Escaneando servos {SERVO_IDS} en {PORT} @ {BAUD}...")
    print("[INFO] Mueve cada articulación manualmente para ver su rango.\n")

    # Inicializar diccionario de rangos
    min_pos = {sid: 9999 for sid in SERVO_IDS}
    max_pos = {sid: 0 for sid in SERVO_IDS}

    try:
        with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
            while True:
                line = []
                for sid in SERVO_IDS:
                    pos = read_position(ser, sid)
                    if pos is None:
                        line.append(f"ID{sid}: ----")
                        continue

                    # Actualiza mínimos y máximos
                    min_pos[sid] = min(min_pos[sid], pos)
                    max_pos[sid] = max(max_pos[sid], pos)

                    line.append(f"ID{sid}: {pos:4d} (min:{min_pos[sid]:4d}, max:{max_pos[sid]:4d})")

                sys.stdout.write("\r" + " | ".join(line) + " " * 10)
                sys.stdout.flush()
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\n[INFO] Calibración detenida por usuario.")
        print("[RESULTADOS FINALES]")
        for sid in SERVO_IDS:
            print(f"Servo {sid}:  min={min_pos[sid]}   max={max_pos[sid]}")
        print("\n[INFO] Usa estos valores para tu joint_limits.yaml.")
    except serial.SerialException:
        print(f"[ERROR] No se puede abrir el puerto {PORT}.")
    except Exception as e:
        print(f"[ERROR] {e}")


if __name__ == "__main__":
    main()
