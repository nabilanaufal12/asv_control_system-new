# backend/extensions.py
# File ini berfungsi sebagai tempat terpusat untuk ekstensi Flask.

from flask_socketio import SocketIO

# Buat instance SocketIO di sini untuk diimpor oleh file lain
socketio = SocketIO(cors_allowed_origins="*")