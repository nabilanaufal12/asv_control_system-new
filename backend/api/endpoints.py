# backend/api/endpoints.py
# Mendefinisikan semua rute API untuk server Flask.

from flask import Blueprint, jsonify, request, Response
import time

# Kita akan menggunakan 'g' dari Flask untuk menyimpan instance handler
# agar bisa diakses di semua rute.
from flask import g

# Membuat 'Blueprint', ini adalah cara Flask untuk mengorganisir rute
api_blueprint = Blueprint('api', __name__)

def generate_video_frames():
    """Generator untuk video streaming."""
    while True:
        # Mengambil frame dari vision_service yang tersimpan di 'g'
        frame = g.vision_service.get_frame()
        if frame is None:
            # Jika tidak ada frame, tunggu sebentar sebelum mencoba lagi
            time.sleep(0.1)
            continue
        
        # Format response untuk streaming (multipart/x-mixed-replace)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@api_blueprint.route('/status', methods=['GET'])
def get_status():
    """Endpoint untuk mendapatkan state terbaru dari ASV."""
    return jsonify(g.asv_handler.get_current_state())

@api_blueprint.route('/command', methods=['POST'])
def handle_command():
    """Endpoint untuk menerima dan memproses perintah umum dari GUI."""
    data = request.json
    g.asv_handler.process_command(data.get("command"), data.get("payload"))
    return jsonify({"status": "command_received"})

@api_blueprint.route('/video_feed')
def video_feed():
    """Endpoint untuk streaming video ke GUI."""
    return Response(generate_video_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@api_blueprint.route('/list_cameras', methods=['GET'])
def list_cameras():
    """Endpoint untuk mendapatkan daftar indeks kamera yang tersedia di backend."""
    cameras = g.vision_service.list_available_cameras()
    return jsonify({"cameras": cameras})

@api_blueprint.route('/vision_command', methods=['POST'])
def handle_vision_command():
    """Endpoint untuk menerima perintah khusus untuk layanan visi."""
    data = request.json
    command = data.get("command")
    payload = data.get("payload")
    
    # Teruskan perintah ke VisionService yang sudah tersimpan di 'g'
    g.vision_service.process_command(command, payload)
    
    return jsonify({"status": "vision_command_received"})