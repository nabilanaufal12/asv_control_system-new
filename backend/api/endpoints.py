# backend/api/endpoints.py

from flask import Blueprint, jsonify, g, current_app

# 1. Impor HANYA socketio dari extensions untuk menghindari impor melingkar
from backend.extensions import socketio

api_blueprint = Blueprint('api', __name__)

@socketio.on('connect')
def handle_connect():
    """
    Fungsi ini sekarang hanya untuk mencatat koneksi baru dari GUI.
    Layanan visi sudah berjalan secara independen sejak backend dimulai.
    """
    print("Klien GUI terhubung.")

@socketio.on('command')
def handle_socket_command(json_data):
    """Menerima dan meneruskan perintah dari GUI."""
    command = json_data.get('command')
    payload = json_data.get('payload')
    print(f"Menerima perintah via WebSocket: {command}")

    # Teruskan perintah ke handler yang sesuai
    # (Menggunakan current_app lebih aman daripada g di dalam event SocketIO)
    if command == 'LIST_CAMERAS':
        # Mengambil daftar kamera dan mengirimkannya kembali ke GUI
        cameras = current_app.vision_service.list_available_cameras()
        socketio.emit('camera_list', {'cameras': cameras})
    elif command.startswith("SELECT_CAMERA") or command.startswith("SET_"):
        # Perintah yang ditujukan untuk VisionService
        current_app.vision_service.process_command(command, payload)
    else:
        # Perintah lainnya untuk AsvHandler
        current_app.asv_handler.process_command(command, payload)

# --- Endpoint HTTP lainnya tidak berubah ---
# Endpoint ini tetap berguna untuk pengujian cepat atau akses langsung via browser
@api_blueprint.route('/list_cameras', methods=['GET'])
def list_cameras():
    """Endpoint HTTP untuk mendapatkan daftar indeks kamera yang tersedia."""
    cameras = g.vision_service.list_available_cameras()
    return jsonify({"cameras": cameras})

@api_blueprint.route('/status', methods=['GET'])
def get_status():
    """Endpoint HTTP untuk mendapatkan state terbaru dari ASV."""
    return jsonify(g.asv_handler.get_current_state())