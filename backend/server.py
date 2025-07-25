# backend/server.py
# VERSI PERBAIKAN: Disederhanakan untuk mengatasi error 404.

from flask import Flask, jsonify, request
import threading
import time

# Impor langsung karena file berada di direktori yang sama
from serial_handler import SerialHandler

# --- Inisialisasi Objek & State Global ---
serial_handler = SerialHandler()
state_lock = threading.Lock()

asv_state = {
    "latitude": 0.0, "longitude": 0.0, "heading": 0.0, "speed": 0.0,
    "battery_voltage": 0.0, "status": "DISCONNECTED", "mission_time": "00:00:00"
}

def parse_serial_data(data_string):
    """Mem-parsing data dari serial, contoh: "LAT:-6.21;LON:106.84" """
    try:
        parts = data_string.split(';')
        with state_lock:
            for part in parts:
                if ':' in part:
                    key, value = part.split(':', 1)
                    key = key.strip()
                    if key == 'LAT': asv_state['latitude'] = float(value)
                    elif key == 'LON': asv_state['longitude'] = float(value)
                    elif key == 'HDG': asv_state['heading'] = float(value)
                    elif key == 'BAT': asv_state['battery_voltage'] = float(value)
    except Exception as e:
        print(f"[Parser] Error: {e}")

def read_from_serial_loop():
    """Loop di background untuk membaca data dari serial."""
    while True:
        if serial_handler.is_connected:
            data = serial_handler.read_data()
            if data:
                print(f"[SerialReader] Menerima: {data}")
                parse_serial_data(data)
        time.sleep(0.05)

# --- Application Factory ---
def create_app():
    """Membuat dan mengkonfigurasi instance aplikasi Flask."""
    app = Flask(__name__)

    @app.route('/status', methods=['GET'])
    def get_status():
        with state_lock:
            return jsonify(asv_state.copy())

    @app.route('/command', methods=['POST'])
    def handle_command():
        data = request.json
        command = data.get("command")
        payload = data.get("payload")
        
        print(f"✅ Perintah diterima: {command}, Payload: {payload}")

        if command == "CONFIGURE_SERIAL" and payload:
            if serial_handler.connect(payload.get("port"), int(payload.get("baudrate"))):
                with state_lock:
                    asv_state['status'] = "IDLE"
        elif command == "MANUAL_CONTROL":
            serial_handler.send_command(f"M:{payload}")
        elif command == "START_MISSION":
            with state_lock:
                asv_state['status'] = "NAVIGATING"
        elif command == "STOP_MISSION":
            with state_lock:
                asv_state['status'] = "IDLE"
            serial_handler.send_command("M:STOP")
        # Tambahkan perintah lain di sini...
        
        return jsonify({"status": "command_received", "command": command})

    return app
