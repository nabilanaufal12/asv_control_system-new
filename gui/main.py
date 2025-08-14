# gui/main.py
# --- MODIFIKASI: Memuat file config.json saat aplikasi dimulai ---

import sys
import os
import json # <-- 1. Impor library json
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QFile, QTextStream

# --- PERBAIKAN PATH ---
try:
    gui_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(gui_dir)
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
except NameError:
    sys.path.insert(0, '.')

# --- IMPOR SETELAH PATH DIPERBAIKI ---
from gui.views.main_window import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)

    # --- 2. MEMUAT FILE KONFIGURASI ---
    config = {} # Buat dictionary kosong sebagai default
    try:
        config_path = os.path.join(project_root, 'config.json')
        with open(config_path, 'r') as f:
            config = json.load(f)
        print("File konfigurasi 'config.json' berhasil dimuat.")
    except FileNotFoundError:
        print("PERINGATAN: File 'config.json' tidak ditemukan. Menggunakan nilai default.")
    except json.JSONDecodeError:
        print("ERROR: Gagal membaca 'config.json'. Pastikan format JSON sudah benar.")

    # Muat dan terapkan stylesheet untuk tema gelap
    try:
        theme_file = QFile(os.path.join(gui_dir, "assets/resources/dark_theme.qss"))
        if theme_file.open(QFile.ReadOnly | QFile.Text):
            stream = QTextStream(theme_file)
            stylesheet = stream.readAll()
            app.setStyleSheet(stylesheet)
            print("Tema gelap berhasil dimuat.")
        else:
            print(f"Tidak dapat membuka file tema: {theme_file.errorString()}")
    except Exception as e:
        print(f"Gagal memuat tema: {e}")

    # --- 3. MENERUSKAN KONFIGURASI KE MAINWINDOW ---
    window = MainWindow(config=config)
    window.show()

    sys.exit(app.exec())