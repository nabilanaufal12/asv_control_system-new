# backend/main.py
# Titik masuk utama untuk backend server.
# Tugasnya hanya menginisialisasi dan menjalankan server Flask.

import json
import os
from flask import Flask, g
from backend.core.asv_handler import AsvHandler
from backend.services.vision_service import VisionService
from backend.api.endpoints import api_blueprint

def create_app():
    """
    Membuat dan mengkonfigurasi instance aplikasi Flask.
    Ini adalah "pabrik" untuk aplikasi kita.
    """
    app = Flask(__name__)

    # Memuat Konfigurasi dari file config.json
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config.json')
        with open(config_path, 'r') as f:
            app.config['ASV_CONFIG'] = json.load(f)
        print("[Server] File 'config.json' berhasil dimuat.")
    except Exception as e:
        print(f"[Server] KRITIS: Gagal memuat config.json. Error: {e}")
        exit()

    # Inisialisasi semua komponen utama (Handler dan Service)
    asv_handler = AsvHandler(app.config['ASV_CONFIG'])
    vision_service = VisionService(app.config['ASV_CONFIG'], asv_handler)

    # Menyimpan instance handler dan service ke dalam 'g' (global context) Flask
    # agar bisa diakses oleh endpoint di file lain.
    @app.before_request
    def before_request():
        g.asv_handler = asv_handler
        g.vision_service = vision_service

    # Daftarkan semua rute API dari file endpoints.py
    app.register_blueprint(api_blueprint)
    
    # Mulai service background (kamera & deteksi)
    print("🚀 Memulai layanan visi...")
    vision_service.start()
    
    return app

if __name__ == "__main__":
    # Buat aplikasi menggunakan "pabrik" di atas
    app = create_app()
    print("🚀 Memulai Backend Server ASV...")
    
    # Jalankan server Flask
    # debug=False dan threaded=True adalah pengaturan yang baik untuk server
    # yang memiliki proses background seperti layanan visi kita.
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)