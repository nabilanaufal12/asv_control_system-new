# gui/main.py
# --- FINAL: Menggunakan simplefilter untuk menyembunyikan semua FutureWarning ---

import sys
import os
import json
import warnings # <-- 1. Impor library 'warnings'

# --- 2. Tambahkan filter yang lebih kuat di sini ---
# Perintah ini akan mengabaikan SEMUA FutureWarning di seluruh aplikasi.
# Ini adalah cara yang lebih "memaksa" dan seharusnya berhasil.
warnings.simplefilter(action='ignore', category=FutureWarning)
# ----------------------------------------------------

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QFile, QTextStream

# --- Blok ini memperbaiki path agar impor dari folder lain berhasil ---
try:
    gui_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(gui_dir)
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
except NameError:
    sys.path.insert(0, '.')

# --- Impor MainWindow setelah path diperbaiki ---
from gui.views.main_window import MainWindow

# --- Titik masuk utama aplikasi ---
if __name__ == "__main__":
    app = QApplication(sys.argv)

    config = {}
    try:
        config_path = os.path.join(project_root, 'config.json')
        with open(config_path, 'r') as f:
            config = json.load(f)
        print("File konfigurasi 'config.json' berhasil dimuat.")
    except FileNotFoundError:
        print("PERINGATAN: File 'config.json' tidak ditemukan. Menggunakan nilai default.")
    except json.JSONDecodeError:
        print("ERROR: Gagal membaca 'config.json'. Pastikan format JSON sudah benar.")

    # Memuat dan menerapkan stylesheet untuk tema gelap
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

    # Membuat jendela utama dan meneruskan data konfigurasi
    window = MainWindow(config=config)
    window.show()

    # Menjalankan aplikasi
    sys.exit(app.exec())