# run_backend_headless.py
import eventlet
import os
import sys
import logging

# Monkey patch harus dieksekusi sebelum impor lainnya
eventlet.monkey_patch()

# Konfigurasi logging
logging.basicConfig(
    level=logging.WARNING,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)

# Menambahkan path src ke sys.path
try:
    project_root = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(project_root, "src")
    if src_path not in sys.path:
        sys.path.insert(0, src_path)
except NameError:
    sys.path.insert(0, "./src")

from navantara_backend.main import create_app  # noqa: E402
from navantara_backend.extensions import socketio  # noqa: E402

app = create_app()

if __name__ == "__main__":
    logging.info("🚀 Menjalankan Backend Server ASV (SECURE LOCAL MODE)...")

    # Ambil konfigurasi
    config = app.config.get("ASV_CONFIG", {})
    backend_config = config.get("backend_connection", {})

    # --- [KEAMANAN: UBAH DEFAULT KE LOCALHOST] ---
    # Jangan gunakan 0.0.0.0 kecuali Anda ingin akses remote dari laptop lain.
    # 127.0.0.1 memastikan hanya aplikasi lokal (GUI di Jetson) yang bisa akses.
    host = backend_config.get("ip_address", "127.0.0.1")

    # Gunakan 5000 sebagai default port
    port = int(backend_config.get("port", 5000))
    # ---------------------------------------------

    print(f"Server aman berjalan di http://{host}:{port}")
    print("Akses dari luar (WiFi/LAN) diblokir secara default.")

    socketio.run(app, host=host, port=port, debug=False)
