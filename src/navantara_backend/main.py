# src/navantara_backend/main.py
import json
import os
import eventlet
import glob

from dataclasses import asdict

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
from flask_cors import CORS

from navantara_backend.extensions import socketio

# from navantara_backend.core.asv_handler import AsvHandler
from navantara_backend.services.vision_service import VisionService
from navantara_backend.api.endpoints import api_blueprint


def create_app():
    """
    Membuat dan mengkonfigurasi instance aplikasi Flask (Application Factory).
    """

    backend_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.dirname(backend_dir)
    web_folder_path = os.path.join(src_dir, "navantara_web")

    app = Flask(
        __name__, template_folder=web_folder_path, static_folder=web_folder_path
    )

    CORS(app)

    # --- [FIX KRITIS CORS: Preflight & Anti-Cache] ---
    @app.before_request
    def handle_options_request():
        # [FIX 1: Handle OPTIONS Preflight Check]
        if request.method == "OPTIONS":
            response = current_app.make_default_options_response()
            # Izinkan semua Origin untuk preflight
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add(
                "Access-Control-Allow-Headers",
                "Content-Type, Access-Control-Allow-Headers, Authorization, X-Requested-With",
            )
            response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            return response

    @app.after_request
    def add_cors_headers(response):
        # [FIX 2: Tambahkan Header ke Semua Respons (GET/POST/SSE)]
        # Memaksa agar file:// origin (WebEngine) diizinkan mengakses API ini
        if "Access-Control-Allow-Origin" not in response.headers:
            response.headers["Access-Control-Allow-Origin"] = "*"

        # Tambahkan header Anti-Cache (Penting untuk SSE/Ajax)
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
        return response

    # ------------------------------------------------

    print(f"[Server] Web template_folder diatur ke: {web_folder_path}")
    print(f"[Server] Web static_folder diatur ke: {web_folder_path}")

    # Muat konfigurasi
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, "..", "..", "config", "config.json")
        config_path = os.path.normpath(config_path)

        with open(config_path, "r") as f:
            app.config["ASV_CONFIG"] = json.load(f)
        print(f"[Server] File 'config.json' berhasil dimuat dari: {config_path}")
    except Exception as e:
        print(f"[Server] KRITIS: Gagal memuat config.json. Error: {e}")
        print(f"[Server] Mencoba di path: {config_path}")
        exit(1)

    asv_conf = app.config["ASV_CONFIG"]
    use_dynamic = asv_conf.get("system_settings", {}).get("use_dynamic_handler", False)

    if use_dynamic:
        print("\n" + "=" * 50)
        print(" [SYSTEM] MODE: DYNAMIC HANDLER (TESTING)")
        print(" [INFO]   Memuat: core/asv_handler_dynamic.py")
        print("=" * 50 + "\n")
        # Import dari file BARU (pastikan file ini ada)
        from navantara_backend.core.asv_handler_dynamic import AsvHandler
    else:
        print("\n" + "=" * 50)
        print(" [SYSTEM] MODE: STABLE HANDLER (PRODUCTION)")
        print(" [INFO]   Memuat: core/asv_handler.py")
        print("=" * 50 + "\n")
        # Import dari file LAMA/STABIL
        from navantara_backend.core.asv_handler import AsvHandler

    # Inisialisasi layanan
    asv_handler = AsvHandler(app.config["ASV_CONFIG"], socketio)
    vision_service = VisionService(app.config["ASV_CONFIG"], asv_handler, socketio)
    asv_handler.vision_service = vision_service
    app.asv_handler = asv_handler
    app.vision_service = vision_service

    @app.before_request
    def before_request():
        # [CATATAN]: Handle OPTIONS request sudah dilakukan di hook terpisah di atas.
        g.asv_handler = current_app.asv_handler
        g.vision_service = current_app.vision_service

    # Daftarkan blueprint API (yang berisi /live_video_feed)
    app.register_blueprint(api_blueprint)

    # --- TAMBAHAN: Route untuk menyajikan monitor1.html ---
    @app.route("/")
    def index():
        return send_from_directory(app.template_folder, "monitor1.html")

    # --------------------------------------------------------

    @app.route("/local")
    def monitor_local():
        return send_from_directory(app.template_folder, "monitor_local.html")

    # --------------------------------------------------------

    @app.route("/debug")
    def debug_telemetry():
        return send_from_directory(app.template_folder, "debug_telemetry.html")

    # === [FIX AKHIR] Rute untuk menyajikan halaman MapView dari HTTP ===
    @app.route("/map_page")
    def serve_map_page():
        # Mengirim file index.html dari template_folder
        # Ini penting agar Origin peta menjadi http://127.0.0.1:5000
        return send_from_directory(app.template_folder, "index.html")

    # === AKHIR FIX AKHIR ===

    # --- MODIFIKASI: RUTE BARU UNTUK MENYAJIKAN GAMBAR/CSV GALERI ---
    @app.route("/logs/<path:filepath>")
    def serve_logs_file(filepath):
        """
        Menyajikan file statis (gambar, csv) dari direktori logs/.
        """
        logs_dir = os.path.join(os.getcwd(), "logs")
        return send_from_directory(logs_dir, filepath)

    @app.route("/api/races", methods=["GET"])
    def list_races():
        """
        Mengembalikan daftar race yang tersedia di folder logs/.
        """
        try:
            logs_dir = os.path.join(os.getcwd(), "logs")
            if not os.path.exists(logs_dir):
                return jsonify([])

            races = []
            for entry in os.listdir(logs_dir):
                if entry.startswith("race_") and os.path.isdir(
                    os.path.join(logs_dir, entry)
                ):
                    try:
                        num = int(entry.split("_")[1])
                        races.append({"id": num, "name": f"Race {num}"})
                    except ValueError:
                        continue

            # Urutkan berdasarkan id desc
            races.sort(key=lambda x: x["id"], reverse=True)
            return jsonify(races)
        except Exception as e:
            print(f"[API Log] Error listing races: {e}")
            return jsonify({"error": str(e)}), 500

    @app.route("/api/get-race-data", methods=["GET"])
    def get_race_data():
        """
        Mengembalikan gabungan telemetri dan daftar gambar untuk suatu race_id.
        """
        try:
            race_id = request.args.get("race_id")
            if not race_id:
                return jsonify({"error": "Missing race_id param"}), 400

            race_dir = os.path.join(os.getcwd(), "logs", f"race_{race_id}")
            if not os.path.exists(race_dir):
                return jsonify({"error": "Race not found"}), 404

            # 1. Parse Telemetry
            csv_path = os.path.join(race_dir, "telemetry", "mission_data.csv")
            telemetry_data = []
            if os.path.exists(csv_path):
                import csv

                with open(csv_path, "r") as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        telemetry_data.append(row)

            # 2. Parse Captures
            captures_dir = os.path.join(race_dir, "captures")
            filenames = []
            if os.path.exists(captures_dir):
                search_path = os.path.join(captures_dir, "*.jpg")
                full_paths = glob.glob(search_path)
                filenames = [os.path.basename(f) for f in full_paths]
                filenames.sort()

            return jsonify({"telemetry": telemetry_data, "captures": filenames})

        except Exception as e:
            print(f"[API Log] Error parsing race data for {race_id}: {e}")
            return jsonify({"error": str(e)}), 500

    # --- 4. TAMBAHAN ENDPOINT API ---
    @app.route("/api/telemetry")
    def api_get_telemetry():
        """
        Endpoint API HTTP untuk mengambil data telemetri terbaru sebagai JSON.
        """
        try:
            state_data = current_app.asv_handler.current_state

            with current_app.asv_handler.state_lock:
                data_copy = asdict(state_data)

            # Sisipkan captures terbaru (opsional untuk REST API, tapi baik untuk konsistensi)
            try:
                captures_dir = current_app.asv_handler.logger.get_current_capture_dir()
                search_path = os.path.join(captures_dir, "*.jpg")
                full_paths = glob.glob(search_path)
                filenames = [os.path.basename(f) for f in full_paths]
                filenames.sort()
                data_copy["captures"] = filenames
            except Exception:
                data_copy["captures"] = []

            response = jsonify(data_copy)

            # Header Anti-Cache sudah diurus oleh @app.after_request
            return response

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
        """
        try:
            data = request.json
            capture_type = data.get("type")

            if capture_type not in ["surface", "underwater"]:
                return (
                    jsonify({"status": "error", "message": "Tipe capture tidak valid"}),
                    400,
                )

            result = current_app.vision_service.trigger_manual_capture(capture_type)

            if result.get("status") == "success":
                return jsonify(result), 200
            else:
                return jsonify(result), 500

        except Exception as e:
            print(f"[API Capture] Error: {e}")
            return jsonify({"status": "error", "message": str(e)}), 500

    # --- AKHIR MODIFIKASI ---

    # --- MODIFIKASI: RUTE BARU UNTUK API GALERI ---
    @app.route("/api/races/<int:race_id>/gallery")
    def get_gallery(race_id):
        """
        Endpoint API HTTP untuk mengambil daftar file gambar galeri dari race tertentu sebagai JSON.
        """
        try:
            captures_dir = os.path.join(
                os.getcwd(), "logs", f"race_{race_id}", "captures"
            )
            search_path = os.path.join(captures_dir, "*.jpg")
            full_paths = glob.glob(search_path)
            filenames = [os.path.basename(f) for f in full_paths]
            filenames.sort()
            return jsonify(filenames)

        except Exception as e:
            return (
                jsonify({"error": str(e), "message": "Gagal mencari galeri."}),
                500,
            )

    @app.route("/api/gallery/live")
    def get_gallery_live():
        try:
            captures_dir = current_app.asv_handler.logger.get_current_capture_dir()
            search_path = os.path.join(captures_dir, "*.jpg")
            full_paths = glob.glob(search_path)
            filenames = [os.path.basename(f) for f in full_paths]
            filenames.sort()
            return jsonify(filenames)

        except Exception as e:
            return (
                jsonify({"error": str(e), "message": "Gagal mencari galeri live."}),
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
                    with current_app.asv_handler.state_lock:
                        data = asdict(current_app.asv_handler.current_state)

                    # Sisipkan nama-nama file gambar terkini ke payload SSE
                    try:
                        captures_dir = (
                            current_app.asv_handler.logger.get_current_capture_dir()
                        )
                        search_path = os.path.join(captures_dir, "*.jpg")
                        full_paths = glob.glob(search_path)
                        filenames = [os.path.basename(f) for f in full_paths]
                        filenames.sort()
                        data["captures"] = filenames
                    except Exception:
                        data["captures"] = []

                    json_data = json.dumps(data)

                    yield f"data: {json_data}\n\n"

                    eventlet.sleep(0.1)
            except Exception:
                pass
            finally:
                print("[SSE Stream] Menghentikan stream telemetri.")

        return Response(
            stream_with_context(telemetry_stream()), content_type="text/event-stream"
        )

    # --- AKHIR DARI ENDPOINT BARU ---

    # Inisialisasi SocketIO
    # [PENTING] Set cors_allowed_origins='*' untuk kompatibilitas WebEngine/file://
    socketio.init_app(app, async_mode="eventlet", cors_allowed_origins="*")

    # Jalankan loop layanan sebagai greenlet
    print("? Menjadwalkan layanan latar belakang sebagai greenlet...")
    eventlet.spawn(asv_handler.main_logic_loop)
    eventlet.spawn(vision_service.run_capture_loops)

    print("[Server] Konfigurasi aplikasi selesai.")
    return app
