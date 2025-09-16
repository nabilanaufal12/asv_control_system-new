# run_backend_headless.py
import eventlet
import os
import sys

# --- PERUBAHAN KRUSIAL: Monkey patch harus dieksekusi sebelum impor lainnya ---
# Baris ini secara dinamis mengganti pustaka standar Python dengan implementasi
# yang kooperatif dari eventlet. Ini adalah kunci untuk memungkinkan ribuan
# green threads berjalan secara efisien dalam satu proses.
eventlet.monkey_patch()

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
    print("🚀 Menjalankan Backend Server ASV dalam Mode Otonom/Headless...")

    # --- PERUBAHAN: Menjalankan server menggunakan eventlet ---
    # Flask-SocketIO akan secara otomatis menggunakan server eventlet karena
    # monkey_patch() telah dipanggil. Ini adalah cara yang direkomendasikan
    # untuk production. Argumen 'allow_unsafe_werkzeug' tidak lagi diperlukan.
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)
