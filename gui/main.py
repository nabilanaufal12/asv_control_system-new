# gui/main.py
# --- MODIFIKASI FINAL: Memindahkan inisialisasi ApiClient untuk koneksi lebih cepat ---

import sys
import os
import json
import warnings

# Mengabaikan FutureWarning yang tidak relevan dari library lain
warnings.simplefilter(action='ignore', category=FutureWarning)

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QFile, QTextStream

# --- Blok ini memperbaiki path agar impor dari folder lain berhasil ---
try:
    gui_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(gui_dir)
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
except NameError:
    # Fallback jika __file__ tidak terdefinisi
    sys.path.insert(0, '.')

# --- Impor setelah path diperbaiki ---
from gui.views.main_window import MainWindow
from gui.api_client import ApiClient # <-- 1. Impor ApiClient di sini

# --- Titik masuk utama aplikasi ---
if __name__ == "__main__":
    app = QApplication(sys.argv)

    config = {}
    try:
        config_path = os.path.join(project_root, 'config.json')
        with open(config_path, 'r') as f:
            config = json.load(f)
        print("File konfigurasi 'config.json' berhasil dimuat.")
    except Exception as e:
        print(f"KRITIS: Gagal memuat 'config.json'. Aplikasi akan berhenti. Error: {e}")
        sys.exit(1) # Keluar jika config tidak bisa dimuat

    # --- 2. Buat instance ApiClient SEBELUM membuat MainWindow ---
    # Ini memastikan thread koneksi ke backend mulai berjalan secepat mungkin,
    # sebelum UI yang berat dimuat.
    api_client = ApiClient(config=config)
    print("ApiClient telah diinisialisasi dan thread polling dimulai.")

    # Memuat dan menerapkan stylesheet untuk tema gelap
    try:
        # Path ke stylesheet bisa disederhanakan
        theme_path = os.path.join(gui_dir, "assets", "resources", "dark_theme.qss")
        with open(theme_path, 'r') as f:
            app.setStyleSheet(f.read())
        print("Tema gelap berhasil dimuat.")
    except Exception as e:
        print(f"Peringatan: Gagal memuat tema gelap. Error: {e}")

    # --- 3. Teruskan instance api_client yang sudah ada ke MainWindow ---
    # MainWindow sekarang tidak perlu membuat ApiClient sendiri.
    window = MainWindow(config=config, api_client=api_client)
    window.show()

    # Menjalankan aplikasi Qt
    sys.exit(app.exec())