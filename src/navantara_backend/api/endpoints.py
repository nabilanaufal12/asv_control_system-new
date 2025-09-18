# src/navantara_backend/api/endpoints.py
from flask import Blueprint, current_app

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
    current_app.vision_service.set_gui_listening(False)
    current_app.asv_handler.set_streaming_status(False)


@socketio.on("request_stream")
def handle_request_stream(json_data):
    """
    Klien meminta server untuk memulai atau menghentikan pengiriman data.
    """
    status = json_data.get("status", True)
    print(
        f"Menerima permintaan stream dari GUI. Mengatur status streaming ke: {status}"
    )
    current_app.vision_service.set_gui_listening(status)
    current_app.asv_handler.set_streaming_status(status)


@socketio.on("command")
def handle_socket_command(json_data):
    """Menerima dan meneruskan perintah umum dari GUI ke layanan yang sesuai."""
    command = json_data.get("command")
    payload = json_data.get("payload", {})
    print(f"Menerima perintah via WebSocket: {command} dengan payload: {payload}")

    # --- PERBAIKAN UTAMA DI SINI ---
    # Definisikan secara eksplisit perintah mana yang untuk vision service
    vision_commands = ["SET_MODE", "SET_INVERSION"]

    if command in vision_commands:
        # Jika perintah ada dalam daftar, kirim ke vision_service
        method_name = command.lower()  # Contoh: "SET_MODE" -> "set_mode"
        if hasattr(current_app.vision_service, method_name):
            # Memanggil fungsi yang sesuai di vision_service, cth: vision_service.set_mode(payload)
            getattr(current_app.vision_service, method_name)(payload)
        else:
            print(f"[API] Perintah visi tidak dikenal: {command}")
    else:
        # Untuk semua perintah lainnya (termasuk SET_WAYPOINTS, NAV_START, dll.)
        # kirim ke asv_handler
        current_app.asv_handler.process_command(command, payload)
    # --- AKHIR PERBAIKAN ---
