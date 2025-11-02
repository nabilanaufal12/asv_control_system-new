# src/navantara_backend/main.py
import json
import os
import eventlet
import time
import glob  # <-- MODIFIKASI: Impor glob untuk pencarian file

# --- TAMBAHAN IMPORT ---
from flask import (
    Flask,
    g,
    current_app,
    send_from_directory,
    jsonify,
    Response,
    stream_with_context,
    request,
)
from flask_cors import CORS  # <--- 2. TAMBAHKAN IMPORT BARU INI

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

    CORS(app)  # <--- 3. INISIALISASI CORS DI SINI

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

    # === TAMBAHKAN KODE BARU INI UNTUK HALAMAN DEBUG ===
    @app.route("/debug")
    def debug_telemetry():
        # Ini akan menyajikan file 'debug_telemetry.html'
        return send_from_directory(app.template_folder, "debug_telemetry.html")

    # === AKHIR KODE BARU ===

    # --- MODIFIKASI: RUTE BARU UNTUK MENYAJIKAN GAMBAR GALERI ---
    @app.route("/captures/<path:filename>")
    def serve_capture(filename):
        """
        Menyajikan file gambar statis dari direktori logs/captures.
        """
        # os.getcwd() adalah root proyek (tempat run_backend_headless.py dijalankan)
        captures_dir = os.path.join(os.getcwd(), "logs", "captures")

        # send_from_directory akan menangani keamanan path
        return send_from_directory(captures_dir, filename)

    # --- AKHIR MODIFIKASI ---

    # --- 4. TAMBAHKAN ENDPOINT API BARU DI SINI ---
    @app.route("/api/telemetry")
    def api_get_telemetry():
        """
        Endpoint API HTTP untuk mengambil data telemetri terbaru sebagai JSON.
        """
        try:
            state_data = current_app.asv_handler.current_state

            with current_app.asv_handler.state_lock:
                data_copy = state_data.copy()

            # Buat respons JSON
            response = jsonify(data_copy)

            # --- TAMBAHAN UNTUK ANTI-CACHE ---
            # Memberi tahu browser untuk tidak menyimpan cache
            response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
            response.headers["Pragma"] = "no-cache"
            response.headers["Expires"] = "0"
            # --- AKHIR TAMBAHAN ---

            return response  # <--- Mengembalikan respons yang sudah dimodifikasi

        except Exception as e:
            return (
                jsonify({"error": str(e), "message": "Gagal mengambil data state."}),
                500,
            )

    # --- AKHIR DARI ENDPOINT BARU ---

    # --- MODIFIKASI: ENDPOINT BARU UNTUK MANUAL CAPTURE ---
    @app.route("/api/capture/manual", methods=["POST"])
    def manual_capture():
        """
        Endpoint API untuk memicu pengambilan gambar manual.
        Menerima JSON: {"type": "surface"} atau {"type": "underwater"}
        """
        try:
            data = request.json
            capture_type = data.get("type")  # 'surface' or 'underwater'

            if capture_type not in ["surface", "underwater"]:
                return (
                    jsonify({"status": "error", "message": "Tipe capture tidak valid"}),
                    400,
                )

            # Panggil fungsi baru di vision_service
            # Ini akan mengambil foto, menambah overlay, dan menyimpannya
            result = current_app.vision_service.trigger_manual_capture(capture_type)

            if result.get("status") == "success":
                return jsonify(result), 200
            else:
                return jsonify(result), 500  # Server error jika gagal

        except Exception as e:
            print(f"[API Capture] Error: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500

    # --- AKHIR MODIFIKASI ---

    # --- MODIFIKASI: RUTE BARU UNTUK API GALERI ---
    @app.route("/api/gallery")
    def get_gallery():
        """
        Endpoint API HTTP untuk mengambil daftar file gambar galeri sebagai JSON.
        """
        try:
            # os.getcwd() adalah root proyek (tempat run_backend_headless.py dijalankan)
            captures_dir = os.path.join(os.getcwd(), "logs", "captures")

            # Cari semua file .jpg di folder itu
            search_path = os.path.join(captures_dir, "*.jpg")

            # Gunakan glob untuk mendapatkan daftar file penuh
            full_paths = glob.glob(search_path)

            # Ambil hanya nama filenya saja (bukan path lengkap)
            filenames = [os.path.basename(f) for f in full_paths]

            # Urutkan berdasarkan nama (opsional, tapi rapi)
            filenames.sort()

            # Kembalikan daftar sebagai JSON
            return jsonify(filenames)

        except Exception as e:
            return (
                jsonify({"error": str(e), "message": "Gagal mencari galeri."}),
                500,
            )

    # --- AKHIR MODIFIKASI ---

    @app.route("/stream-telemetry")
    def stream_telemetry():
        """
        Endpoint streaming Server-Sent Events (SSE) untuk telemetri.
        """

        def telemetry_stream():
            """Generator untuk stream data."""
            try:
                while True:
                    # Ambil data telemetri terbaru dari asv_handler
                    # Ini adalah cara thread-safe (greenlet-safe)
                    with current_app.asv_handler.state_lock:
                        data = current_app.asv_handler.current_state.copy()

                    # Format data sebagai JSON
                    json_data = json.dumps(data)

                    # Kirim data dalam format SSE: "data: ...\n\n"
                    yield f"data: {json_data}\n\n"

                    # Kirim data 10 kali per detik (sesuai loop utama asv_handler)
                    eventlet.sleep(0.1)
            except Exception as e:
                # Tangani jika klien terputus
                print(f"[SSE Stream] Klien terputus: {e}")
            finally:
                print("[SSE Stream] Menghentikan stream telemetri.")

        # Kembalikan Response streaming
        return Response(
            stream_with_context(telemetry_stream()), content_type="text/event-stream"
        )

    # --- AKHIR DARI ENDPOINT BARU ---

    # Inisialisasi SocketIO
    socketio.init_app(app, async_mode="eventlet", cors_allowed_origins="*")

    # Jalankan loop layanan sebagai greenlet
    print("? Menjadwalkan layanan latar belakang sebagai greenlet...")
    eventlet.spawn(asv_handler.main_logic_loop)
    eventlet.spawn(vision_service.run_capture_loops)

    print("[Server] Konfigurasi aplikasi selesai.")
    return app
