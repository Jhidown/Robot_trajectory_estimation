# ============================================================
# Author      : Baptiste Poncet
# Date        : 07/07/2025
# File        : serial_comm.py
# Description : Serial communication with the robot via LoRa transmittion
# ============================================================


import serial
import threading
import time
from datetime import datetime

# ===== Stores the serial object and read status in the module (internal) =====
_serial_obj = None
_reading = False

def start_connection(port: str, baudrate: int, on_message_callback, on_status_callback) -> bool:
    """
    Initializes the serial connection and starts reading in a thread.
    - on_message_callback(message: str)` is called for each message received.
    - on_status_callback(text: str, color: str)` is called for status (GUI).
    """
    global _serial_obj, _reading

    try:
        _serial_obj = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Allow time for the connection to be established
        _reading = True
        threading.Thread(target=_read_from_port, args=(on_message_callback,), daemon=True).start()

        on_status_callback(f"Connected to {port} at {baudrate} bauds.", "green")
        return True

    except serial.SerialException as e:
        on_status_callback(f"Connection error: {e}", "red")
        return False

def _read_from_port(on_message_callback):
    """
    Serial read loop in a separate thread.
    """
    global _serial_obj, _reading

    while _reading:
        if _serial_obj and _serial_obj.in_waiting:
            try:
                line = _serial_obj.readline().decode('utf-8').strip()
                if line:
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    on_message_callback(f"[{timestamp}][LoRa ➜ PC] {line}")
            except Exception as e:
                on_message_callback(f"[{datetime.now().strftime('%H:%M:%S')}][ERROR] {e}")
                break
        time.sleep(0.1)

def send_custom_message(message: str, on_status_callback=None):
    """
    Sends a message via the serial port.
    """
    global _serial_obj
    if _serial_obj and _serial_obj.is_open:
        try:
            _serial_obj.write((message + "\n").encode("utf-8"))
            if on_status_callback:
                on_status_callback(f"{message} sent", "green")
        except Exception as e:
            if on_status_callback:
                on_status_callback(f"Send error: {e}", "red")
    else:
        if on_status_callback:
            on_status_callback("Serial port not connected", "red")

def close_connection(on_status_callback=None):
    """
    Closes the serial connection cleanly.
    """
    global _serial_obj, _reading
    _reading = False
    if _serial_obj and _serial_obj.is_open:
        _serial_obj.close()
        if on_status_callback:
            on_status_callback(" Connection closed.", "orange")
