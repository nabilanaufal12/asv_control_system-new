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
    # --- MODIFIKASI: Tentukan static_folder dan template_folder ---
    # Dapatkan path absolut ke direktori 'navantara_backend'
    backend_dir = os.path.dirname(os.path.abspath(__file__))
    # Tentukan path ke folder ASV_MONITOR relatif terhadap backend_dir
    monitor_dir = os.path.join(backend_dir, "ASV_MONITOR")

    app = Flask(__name__,
            template_folder=os.path.abspath(
                'src/navantara_web'), # <-- UBAH INI
            static_folder=os.path.abspath(
                'src/navantara_web')  # <-- UBAH INI
            )
    # -------------------------------------------------------------

    # Muat konfigurasi
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "..", "..", "config", "config.json")
        with open(config_path, "r") as f:
            app.config["ASV_CONFIG"] = json.load(f)
        print("[Server] File 'config.json' berhasil dimuat.")
    except Exception as e:
        print(f"[Server] KRITIS: Gagal memuat config.json. Error: {e}")
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
        # (Flask secara otomatis mencari di folder yang ditentukan di atas)
        # Jika Anda tidak memindahkannya, gunakan: return send_from_directory('../../ASV_MONITOR', 'monitor1.html')
        return send_from_directory(app.template_folder, "monitor1.html")

    # --------------------------------------------------------

    # --- TAMBAHKAN KODE BARU ANDA DI SINI ---
    @app.route("/local")
    def monitor_local():
        # Ini akan menyajikan file 'monitor_local.html' yang baru Anda buat
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
