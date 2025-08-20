# backend/api/endpoints.py
# --- MODIFIKASI: Mengimpor socketio dari extensions.py untuk mengatasi Circular Import ---

import time
from flask import Blueprint, jsonify, request, Response, g, current_app
# 1. Impor instance socketio dari file extensions.py yang baru, BUKAN dari main.py
from backend.extensions import socketio

# Membuat 'Blueprint' untuk mengorganisir rute HTTP (tidak berubah)
api_blueprint = Blueprint('api', __name__)

# --- 2. Event handler Socket.IO sekarang menggunakan socketio yang diimpor dari sumber yang benar ---
@socketio.on('command')
def handle_socket_command(json_data):
    """
    Menerima perintah dari GUI melalui koneksi WebSocket.
    """
    command = json_data.get('command')
    payload = json_data.get('payload')
    print(f" Menerima perintah via WebSocket: {command}")
    
    # Menggunakan 'g' untuk mengakses handler yang sudah ada di konteks request
    if hasattr(g, 'asv_handler'):
        g.asv_handler.process_command(command, payload)

# --- Endpoint HTTP yang lain tidak perlu diubah ---

def generate_video_frames():
    """Generator untuk video streaming."""
    vision_service = current_app.vision_service
    while True:
        frame = vision_service.get_frame()
        if frame is None:
            time.sleep(0.05)
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@api_blueprint.route('/video_feed')
def video_feed():
    """Endpoint untuk streaming video ke GUI."""
    return Response(generate_video_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@api_blueprint.route('/list_cameras', methods=['GET'])
def list_cameras():
    """Endpoint untuk mendapatkan daftar indeks kamera yang tersedia."""
    cameras = g.vision_service.list_available_cameras()
    return jsonify({"cameras": cameras})

# Endpoint ini bisa disimpan untuk keperluan debugging atau akses langsung via browser
@api_blueprint.route('/status', methods=['GET'])
def get_status():
    """Endpoint untuk mendapatkan state terbaru dari ASV."""
    return jsonify(g.asv_handler.get_current_state())