# backend/main.py
# Titik masuk untuk menjalankan server backend.

import threading
# Pastikan Anda menginstal flask: pip install flask
from server import create_app, read_from_serial_loop

if __name__ == "__main__":
    # Buat instance aplikasi menggunakan factory
    app = create_app()

    # Jalankan thread untuk membaca data dari serial di background
    serial_reader_thread = threading.Thread(target=read_from_serial_loop, daemon=True)
    serial_reader_thread.start()
    
    print("🚀 Memulai Backend Server ASV...")
    print("   -> Server berjalan di http://0.0.0.0:5000")
    print("   -> Tekan CTRL+C untuk menghentikan.")
    
    # Jalankan server Flask
    app.run(host='0.0.0.0', port=5000, debug=False)
