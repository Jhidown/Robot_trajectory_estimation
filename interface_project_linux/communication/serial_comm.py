# ============================================================
# Author      : Baptiste Poncet
# Last Update : 18/08/2025
# File        : serial_comm.py
# Description : Serial communication utilities
# ============================================================


import serial
import threading
import time
from datetime import datetime
import queue

# ===== Stores the serial object and read status in the module (internal) =====
_serial_obj = None
_reading = False
_on_message_callback = None
_on_status_callback = None
dernier_381 = 0
dernier_382 = 0
_message_queue = queue.Queue()

def start_connection(port: str, baudrate: int, on_message_callback, on_status_callback) -> bool:
    """
    Initializes the serial connection and starts reading in a thread.
    - on_message_callback(message: str)` is called for each message received.
    - on_status_callback(text: str, color: str)` is called for status (GUI).
    """
    global _serial_obj, _reading, _on_message_callback, _on_status_callback

    _on_message_callback = on_message_callback
    _on_status_callback = on_status_callback

    try:
        _serial_obj = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # temps de stabilisation
        _reading = True
        threading.Thread(target=_read_from_port, daemon=True).start()

        if callable(_on_status_callback):
            _on_status_callback(f"Connected to {port} at {baudrate} bauds.", "green")
        return True

    except serial.SerialException as e:
        if callable(_on_status_callback):
            _on_status_callback(f"Connection error: {e}", "red")
        return False

def construire_trame(can_id, data):
    dlc = len(data)
    if dlc > 8:
        raise ValueError("Max 8 octets de données")
    control = 0xC0 | dlc
    trame = bytearray([0xAA, control, can_id & 0xFF, (can_id >> 8) & 0xFF])
    trame.extend(data)
    trame.append(0x55)
    return trame

def envoyer_message(ser, can_id, data):
    trame = construire_trame(can_id, data)
    ser.write(trame)
    print(f"[TX] {can_id:03X} [{' '.join(f'{b:02X}' for b in data)}]")

def lire_trames(buffer):
    """
    Lit toutes les trames CAN valides depuis un buffer persistant.
    - Le buffer est modifié en place (les trames lues sont supprimées).
    - Retourne un générateur de trames brutes (bytes).
    """
    while True:
        # Cherche le début de trame (0xAA)
        start = buffer.find(b'\xAA')
        if start == -1:
            buffer.clear()
            break

        # Pas assez de données pour lire control byte + ID
        if len(buffer) < start + 3:
            break

        # Longueur des données dans la trame
        length = buffer[start + 1] & 0x0F
        trame_len = 2 + 2 + length + 1  # AA + control + ID(2) + data + 55

        # Attendre si la trame n'est pas encore complète
        if len(buffer) < start + trame_len:
            break

        trame = buffer[start:start + trame_len]

        # Vérifie que la trame se termine bien par 0x55
        if trame[-1] == 0x55:
            yield trame

        # Supprime la trame traitée du buffer
        del buffer[:start + trame_len]


def decoder_trame(trame):
    control = trame[1]
    dlc = control & 0x0F
    can_id = trame[2] | (trame[3] << 8)
    data = trame[4:4 + dlc]
    return can_id, data






    

def _read_from_port():
    """
    Lit les trames CAN depuis le port série et appelle _on_message_callback
    pour chaque message filtré + stocke les trames pour wait_for_response.
    """
    global _serial_obj, _reading, dernier_381, dernier_382, _on_message_callback, _message_queue

    buffer = bytearray()

    while _reading:
        if _serial_obj and _serial_obj.in_waiting:
            try:
                data = _serial_obj.read(_serial_obj.in_waiting)
                buffer.extend(data)

                for trame in lire_trames(buffer):
                    can_id, payload = decoder_trame(trame)

                    #  On met toutes les trames reçues dans la queue
                    _message_queue.put((can_id, payload))

                    #  On filtre les messages 0x381 et 0x382
                    if can_id == 0x381:
                        maintenant1 = time.time()
                        if maintenant1 - dernier_381 < 0.5:
                            continue
                        dernier_381 = maintenant1

                    if can_id == 0x382:
                        maintenant2 = time.time()
                        if maintenant2 - dernier_382 < 0.5:
                            continue
                        dernier_382 = maintenant2

                    if can_id in (0x381, 0x382):
                        payload_str = " ".join(f"{b:02X}" for b in payload)
                        if callable(_on_message_callback):
                            _on_message_callback(f"[ENC] ID={can_id:03X} DATA=[{payload_str}]")
                    
            except Exception as e:
                if callable(_on_message_callback):
                    _on_message_callback(f"[ERROR] {e}")
                break

        time.sleep(0.01)


def wait_for_response(expected_id, timeout=1.0):
    """
    Attend une trame avec l'ID attendu dans le délai imparti.
    Retourne le payload (bytes) si reçu, sinon None.
    """
    start = time.time()
    while time.time() - start < timeout:
        try:
            can_id, payload = _message_queue.get(timeout=timeout - (time.time() - start))
            if can_id == expected_id:
                return payload
        except queue.Empty:
            break
    return None

def send_custom_message(can_id, data):
    """
    Envoi d'une trame CAN via le port série.
    """
    global _serial_obj, _on_status_callback
    if _serial_obj and _serial_obj.is_open:
        try:
            envoyer_message(_serial_obj, can_id, data)
            if callable(_on_status_callback):
                _on_status_callback(
                    f"[TX] {can_id:03X} [{' '.join(f'{b:02X}' for b in data)}]", "green"
                )
        except Exception as e:
            if callable(_on_status_callback):
                _on_status_callback(f"Send error: {e}", "red")
    else:
        if callable(_on_status_callback):
            _on_status_callback("Serial port not connected", "red")
    time.sleep(0.1)


def close_connection():
    """
    Closes the serial connection cleanly.
    """
    global _serial_obj, _reading, _on_status_callback
    _reading = False
    if _serial_obj and _serial_obj.is_open:
        _serial_obj.close()
        if callable(_on_status_callback):
            _on_status_callback("Connection closed.", "orange")



def init_motor():
    """
    Initialise les deux moteurs via CAN.
    """
    steps = [
        (0x000, [0x01, 0x01]),
        (0x000, [0x01, 0x02]),
        "pause",

        (0x601, [0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00]),
        (0x602, [0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00]),
        "pause",

        (0x601, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]),
        (0x602, [0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00]),
        "pause",

        (0x601, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]),
        (0x602, [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]),
        "pause",

        (0x601, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]),
        (0x602, [0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00]),
        "pause",

        (0x601, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]),
        (0x602, [0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00]),
        "pause"
    ]

    for step in steps:
        if step == "pause":
            time.sleep(1)
        else:
            can_id, data = step
            send_custom_message(can_id, data)

def send_message_confirm(can_id,data, timeout=0.5):
    send_custom_message(can_id, data)
    node_id = can_id - 0x600
    expected_id= 0x580 + node_id
    response = wait_for_response(expected_id, timeout)
    if response is None:
        print(f"[ERROR] Pas de réponse moteur {node_id}")
        return False
    if response[0] != 0x60:
        print(f"[ERROR] Réponse invalide moteur {node_id}: {response.hex()}")
        return False
    return True

