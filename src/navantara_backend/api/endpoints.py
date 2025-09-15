# src/navantara_backend/api/endpoints.py
from flask import Blueprint, current_app

# 1. Impor HANYA socketio dari extensions untuk menghindari impor melingkar
from navantara_backend.extensions import socketio

api_blueprint = Blueprint("api", __name__)


@socketio.on("connect")
def handle_connect():
    """
    Mencatat koneksi baru dari klien GUI. Server sekarang menunggu
    permintaan eksplisit dari klien untuk memulai streaming data.
    """
    print("Klien GUI terhubung. Menunggu permintaan stream...")


@socketio.on("disconnect")
def handle_disconnect():
    """
    Menangani saat klien terputus, dan memastikan semua stream data berhenti
    untuk menghemat sumber daya.
    """
    print("Klien GUI terputus. Menghentikan semua stream data ke klien ini.")
    # Matikan streaming untuk mencegah pengiriman data yang tidak perlu
    current_app.vision_service.set_gui_listening(False)
    current_app.asv_handler.set_streaming_status(False)


# --- PERUBAHAN UTAMA: Event handler baru untuk pola "pull" ---
@socketio.on('request_stream')
def handle_request_stream(json_data):
    """
    Klien meminta server untuk memulai atau menghentikan pengiriman data.
    Ini adalah inti dari arsitektur komunikasi yang baru.
    """
    status = json_data.get('status', True)
    print(f"Menerima permintaan stream dari GUI. Mengatur status streaming ke: {status}")
    # Meneruskan permintaan ke kedua layanan yang relevan
    current_app.vision_service.set_gui_listening(status)
    current_app.asv_handler.set_streaming_status(status)


@socketio.on("command")
def handle_socket_command(json_data):
    """Menerima dan meneruskan perintah umum dari GUI ke layanan yang sesuai."""
    command = json_data.get("command")
    payload = json_data.get("payload", {})
    print(f"Menerima perintah via WebSocket: {command} dengan payload: {payload}")

    # Menggunakan current_app lebih aman daripada g di dalam event SocketIO
    if command.startswith("SET_"):
        # Perintah yang ditujukan untuk VisionService (misal: SET_MODE)
        # Kita buat nama metode yang konsisten: set_nama_perintah
        method_name = command.lower()
        if hasattr(current_app.vision_service, method_name):
            getattr(current_app.vision_service, method_name)(payload)
        else:
            print(f"[API] Perintah visi tidak dikenal: {command}")
    else:
        # Perintah lainnya diteruskan ke AsvHandler
        current_app.asv_handler.process_command(command, payload)

# --- PERUBAHAN: Endpoint HTTP tidak lagi diperlukan ---
# Komunikasi sekarang sepenuhnya ditangani melalui WebSocket untuk konsistensi
# dan efisiensi, sehingga endpoint ini dapat dihapus.