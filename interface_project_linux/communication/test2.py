import serial
import time
import threading

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

# -------------------- CONSTRUCTION ET ENVOI --------------------
def construire_trame(can_id, data):
    dlc = len(data)
    if dlc > 8:
        raise ValueError("Max 8 octets de données")
    control = 0xC0 | dlc  # standard frame + data frame + DLC
    trame = bytearray([0xAA, control, can_id & 0xFF, (can_id >> 8) & 0xFF])
    trame.extend(data)
    trame.append(0x55)
    return trame

def envoyer_message(ser, can_id, data):
    trame = construire_trame(can_id, data)
    ser.write(trame)
    print(f"[TX] {can_id:03X} [{' '.join(f'{b:02X}' for b in data)}]")

# -------------------- LECTURE ET DÉCODAGE --------------------
def lire_trames(ser):
    buffer = bytearray()
    while True:
        data = ser.read(64)
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

            length = buffer[start + 1] & 0x0F
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
    can_id = trame[2] | (trame[3] << 8)
    data = trame[4:4 + dlc]
    return can_id, data

def lecture_en_continue(ser):
    for trame in lire_trames(ser):
        can_id, data = decoder_trame(trame)
        data_str = " ".join(f"{b:02X}" for b in data)
        print(f"[RX] {can_id:03X} [{data_str}]")

# -------------------- MAIN --------------------
def main():
    with serial.Serial(PORT, BAUDRATE, timeout=0.1) as ser:
        # Thread pour écouter en continu
        thread_rx = threading.Thread(target=lecture_en_continue, args=(ser,), daemon=True)
        thread_rx.start()

        # ---------------- ENVOIS EXEMPLE ----------------
        envoyer_message(ser, 0x000, [0x01, 0x01])
        envoyer_message(ser, 0x000, [0x01, 0x02])
        time.sleep(1)
        
        envoyer_message(ser, 0x601, [0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00])
        envoyer_message(ser, 0x602, [0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00])
        time.sleep(1)

        # speed mode
        envoyer_message(ser, 0x601, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
        envoyer_message(ser, 0x602, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00])
        time.sleep(1)

        # ready to SWO
        envoyer_message(ser, 0x601, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
        envoyer_message(ser, 0x602, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00])
        time.sleep(1)

        # SWO
        envoyer_message(ser, 0x601, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00])
        envoyer_message(ser, 0x602, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00])
        time.sleep(1)

        # Enabled
        envoyer_message(ser, 0x601, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        envoyer_message(ser, 0x602, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        time.sleep(1)

        # SPEED 15000 / -15000
        envoyer_message(ser, 0x602, [0x23, 0xFF, 0x60, 0x00, 0x98, 0x3A, 0x00, 0x00])
        envoyer_message(ser, 0x601, [0x23, 0xFF, 0x60, 0x00, 0x68, 0xC5, 0xFF, 0xFF])
        time.sleep(5)

        # Stop motors
        envoyer_message(ser, 0x602, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        envoyer_message(ser, 0x601, [0x23, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        


        # Laisser tourner la lecture
        while True:
            time.sleep(0.1)

if __name__ == "__main__":
    main()
