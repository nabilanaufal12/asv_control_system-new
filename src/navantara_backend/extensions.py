# src/navantara_backend/extensions.py

from flask_socketio import SocketIO

# Inisialisasi ekstensi SocketIO di sini
# agar bisa diimpor oleh modul lain tanpa menyebabkan circular import.
socketio = SocketIO()