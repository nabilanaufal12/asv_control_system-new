# run_backend_headless.py
import eventlet
import os
import sys
import logging  # <-- [TAMBAHAN 1] Impor modul logging

# --- PERUBAHAN KRUSIAL: Monkey patch harus dieksekusi sebelum impor lainnya ---
eventlet.monkey_patch()

# --- [TAMBAHAN 2] KONFIGURASI LOGGING ---
# Ini adalah perbaikan untuk masalah "tidak ada output" di video Anda.
# Ini harus dijalankan SEBELUM create_app() dipanggil.
logging.basicConfig(
    level=logging.INFO,  # Set level ke INFO agar log .info() muncul
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
# -----------------------------------------

# Menambahkan path src ke sys.path agar impor dari backend berhasil
try:
    project_root = os.path.dirname(os.path.abspath(__file__))
    src_path = os.path.join(project_root, "src")
    if src_path not in sys.path:
        sys.path.insert(0, src_path)
except NameError:
    # Fallback jika dijalankan di lingkungan yang tidak memiliki __file__
    sys.path.insert(0, "./src")

from navantara_backend.main import create_app
from navantara_backend.extensions import socketio

app = create_app()

if __name__ == "__main__":
    # --- [TAMBAHAN 3] Ganti print() ke logging.info() ---
    logging.info("🚀 Menjalankan Backend Server ASV dalam Mode Otonom/Headless...")

    # --- PERUBAHAN: Menjalankan server menggunakan eventlet ---
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)