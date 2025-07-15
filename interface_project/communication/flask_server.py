# ============================================================
# Author      : Baptiste Poncet
# Date        : 07/07/2025
# File        : flask_server.py
# Description : Flask server to receive external commands via HTTP
# ============================================================


from flask import Flask, request, jsonify
from datetime import datetime
import threading

# ===== Flask setup =====
flask_app = Flask(__name__)
FLASK_TOKEN = "mon_token_super_secret"

# ===== Command callback must be registered externally =====
_command_callback = None
_log_callback = None

def register_command_handler(callback):
    global _command_callback
    _command_callback = callback

def register_log_handler(callback):
    global _log_callback
    _log_callback = callback

@flask_app.route("/send", methods=["POST"])
def receive_command():
    global _command_callback, _log_callback

    data = request.get_json()

    if not data:
        return jsonify({"error": "No data received"}), 400

    if data.get("token") != FLASK_TOKEN:
        return jsonify({"error": "Invalid token"}), 403

    responses = []

    # One command
    if "command" in data:
        commands = [data["command"]]
    # Multiple commands
    elif "commands" in data and isinstance(data["commands"], list):
        commands = data["commands"]
    else:
        return jsonify({"error": "Missing command or commands"}), 400

    for command in commands:
        timestamp = datetime.now().strftime("%H:%M:%S")

        if _log_callback:
            _log_callback(f"[{timestamp}][📱 ➜ PC] {command}")

        if _command_callback:
            handled = _command_callback(command)
            if not handled:
                responses.append({"command": command, "status": "unrecognised"})
            else:
                responses.append({"command": command, "status": "executed"})
        else:
            return jsonify({"error": "No command handler registered"}), 500

    return jsonify({"results": responses}), 200

def run_flask_server():
    flask_app.run(host="0.0.0.0", port=5000)

def start_flask_server():
    threading.Thread(target=run_flask_server, daemon=True).start()
