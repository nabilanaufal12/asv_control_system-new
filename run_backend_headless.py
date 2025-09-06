# run_backend_headless.py
import os
import sys

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
    # Gunakan eventlet atau gevent untuk production, atau werkzeug untuk debug
    socketio.run(
        app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True
    )
