# run_backend_headless.py
import eventlet
import os
import sys
import logging

# Monkey patch harus dieksekusi sebelum impor lainnya
eventlet.monkey_patch()

# Konfigurasi logging (Level WARNING agar tidak terlalu berisik)
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

from navantara_backend.main import create_app
from navantara_backend.extensions import socketio

app = create_app()

if __name__ == "__main__":
    logging.info("🚀 Menjalankan Backend Server ASV dalam Mode Otonom/Headless...")

    # --- [PERBAIKAN: AMBIL DARI CONFIG.JSON] ---
    # Ambil konfigurasi yang sudah dimuat oleh create_app()
    config = app.config.get("ASV_CONFIG", {})
    backend_config = config.get("backend_connection", {})

    # Gunakan 0.0.0.0 sebagai default host
    host = backend_config.get("ip_address", "0.0.0.0")
    # Gunakan 5000 sebagai default port
    port = int(backend_config.get("port", 5000))
    # --- [AKHIR PERBAIKAN] ---

    print(f"Server akan berjalan di http://{host}:{port}")
    socketio.run(app, host=host, port=port, debug=False)
