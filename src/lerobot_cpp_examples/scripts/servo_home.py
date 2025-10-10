import serial, time

PORT="/dev/ttyACM0"
BAUD=1_000_000

def checksum(s): return (~(s & 0xFF)) & 0xFF
def pkt(sid,inst,params):
    b=[sid,len(params)+2,inst]+params
    return bytes([0xFF,0xFF]+b+[checksum(sum(b))])

with serial.Serial(PORT,BAUD,timeout=0.05) as ser:
    ser.write(pkt(1,0x01,[]))   # PING ID 1
    ser.flush()
    resp=ser.read(8)
    print("PING resp:", list(resp))
