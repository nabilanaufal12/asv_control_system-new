# src/navantara_backend/main.py
import json
import os
import eventlet

# --- TAMBAHAN IMPORT ---
from flask import Flask, g, current_app, send_from_directory

# -----------------------

from navantara_backend.extensions import socketio
from navantara_backend.core.asv_handler import AsvHandler
from navantara_backend.services.vision_service import VisionService
from navantara_backend.api.endpoints import api_blueprint


def create_app():
    """
    Membuat dan mengkonfigurasi instance aplikasi Flask (Application Factory).
    """

    # --- MODIFIKASI: Perbaikan Path Absolut ---
    # Dapatkan path absolut ke direktori 'main.py' ini
    # (e.g., /home/navantara/navantara/src/navantara_backend)
    backend_dir = os.path.dirname(os.path.abspath(__file__))

    # Dapatkan path ke direktori 'src' (satu level di atas backend_dir)
    # (e.g., /home/navantara/navantara/src)
    src_dir = os.path.dirname(backend_dir)

    # Tentukan path ke folder 'navantara_web'
    # (e.g., /home/navantara/navantara/src/navantara_web)
    web_folder_path = os.path.join(src_dir, "navantara_web")

    # Gunakan path yang benar dan absolut untuk template_folder dan static_folder
    app = Flask(
        __name__, template_folder=web_folder_path, static_folder=web_folder_path
    )

    print(f"[Server] Web template_folder diatur ke: {web_folder_path}")
    print(f"[Server] Web static_folder diatur ke: {web_folder_path}")
    # --- AKHIR MODIFIKASI ---

    # Muat konfigurasi
    try:
        # Path config relatif dari 'main.py'
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "..", "..", "config", "config.json")
        config_path = os.path.normpath(config_path)  # Membersihkan path (../..)

        with open(config_path, "r") as f:
            app.config["ASV_CONFIG"] = json.load(f)
        print(f"[Server] File 'config.json' berhasil dimuat dari: {config_path}")
    except Exception as e:
        print(f"[Server] KRITIS: Gagal memuat config.json. Error: {e}")
        print(f"[Server] Mencoba di path: {config_path}")
        exit(1)

    # Inisialisasi layanan
    asv_handler = AsvHandler(app.config["ASV_CONFIG"], socketio)
    vision_service = VisionService(app.config["ASV_CONFIG"], asv_handler, socketio)
    app.asv_handler = asv_handler
    app.vision_service = vision_service

    @app.before_request
    def before_request():
        g.asv_handler = current_app.asv_handler
        g.vision_service = current_app.vision_service

    # Daftarkan blueprint API (yang berisi /live_video_feed)
    app.register_blueprint(api_blueprint)

    # --- TAMBAHAN: Route untuk menyajikan monitor1.html ---
    @app.route("/")
    def index():
        # Mengirim file monitor1.html dari template_folder
        return send_from_directory(app.template_folder, "monitor1.html")

    # --------------------------------------------------------

    # --- TAMBAHKAN KODE BARU ANDA DI SINI ---
    @app.route("/local")
    def monitor_local():
        # Ini akan menyajikan file 'monitor_local.html'
        return send_from_directory(app.template_folder, "monitor_local.html")

    # --------------------------------------------------------

    # Inisialisasi SocketIO
    socketio.init_app(app, async_mode="eventlet", cors_allowed_origins="*")

    # Jalankan loop layanan sebagai greenlet
    print("? Menjadwalkan layanan latar belakang sebagai greenlet...")
    eventlet.spawn(asv_handler.main_logic_loop)
    eventlet.spawn(vision_service.run_capture_loops)

    print("[Server] Konfigurasi aplikasi selesai.")
    return app
