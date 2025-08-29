import serial

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

def lire_trames(ser):
    buffer = bytearray()
    while True:
        data = ser.read(64)  # lire un bloc
        if not data:
            continue
        buffer.extend(data)

        while True:
            start = buffer.find(b'\xAA')
            if start == -1:
                buffer.clear()
                break

            if len(buffer) < start + 3:
                break

            length = buffer[start + 1] & 0x0F  # 4 bits bas => nb d'octets de data
            trame_len = 2 + 2 + length + 1  # AA + control + ID(2) + data + 55

            if len(buffer) < start + trame_len:
                break

            trame = buffer[start:start + trame_len]

            if trame[-1] == 0x55:
                yield trame

            del buffer[:start + trame_len]

def decoder_trame(trame):
    control = trame[1]
    dlc = control & 0x0F
    can_id = trame[2] | (trame[3] << 8)  # petit endian
    data = trame[4:4 + dlc]
    return can_id, data

def main():
    with serial.Serial(PORT, BAUDRATE, timeout=0.1) as ser:
        print(f"Lecture sur {PORT} @ {BAUDRATE}")
        for trame in lire_trames(ser):
            can_id, data = decoder_trame(trame)
            data_str = " ".join(f"{b:02X}" for b in data)
            print(f"{can_id:03X} [{data_str}]")

if __name__ == "__main__":
    main()
