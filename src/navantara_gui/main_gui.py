# src/navantara_gui/main_gui.py
# --- VERSI FINAL: Dengan path config yang diperbaiki ---

import sys
import os
import json
import warnings

# Mengabaikan FutureWarning yang tidak relevan dari library lain
warnings.simplefilter(action="ignore", category=FutureWarning)

from PySide6.QtWidgets import QApplication

# --- Blok ini memperbaiki path agar impor dari folder lain berhasil ---
try:
    # Menambahkan direktori 'src' ke path
    gui_dir = os.path.dirname(os.path.abspath(__file__))
    src_dir = os.path.dirname(gui_dir)
    if src_dir not in sys.path:
        sys.path.insert(0, src_dir)
except NameError:
    # Fallback jika __file__ tidak terdefinisi
    sys.path.insert(0, ".")

# --- Impor setelah path diperbaiki ---
# Menggunakan import absolut dari nama paket
from navantara_gui.views.main_window import MainWindow


# --- FUNGSI BARU UNTUK MENJALANKAN APLIKASI ---
def run():
    """
    Inisialisasi dan menjalankan seluruh aplikasi GUI.
    """
    app = QApplication(sys.argv)

    config = {}
    try:
        # --- PERBAIKAN UTAMA DI SINI ---
        # Menemukan path root proyek secara dinamis dari lokasi file ini
        gui_dir = os.path.dirname(os.path.abspath(__file__))
        # Dari 'src/navantara_gui', kita naik DUA tingkat untuk mencapai root proyek
        project_root = os.path.dirname(os.path.dirname(gui_dir)) 
        config_path = os.path.join(project_root, "config", "config.json")
        # --- AKHIR PERBAIKAN ---

        with open(config_path, "r") as f:
            config = json.load(f)
        print("File konfigurasi 'config.json' berhasil dimuat.")
    except Exception as e:
        print(f"KRITIS: Gagal memuat 'config.json'. Aplikasi akan berhenti. Error: {e}")
        sys.exit(1)

    # Memuat tema default (terang)
    try:
        theme_path = os.path.join(gui_dir, "assets", "resources", "light_theme.qss")
        with open(theme_path, "r") as f:
            app.setStyleSheet(f.read())
        print("Tema terang berhasil dimuat sebagai default.")
    except Exception as e:
        print(f"Peringatan: Gagal memuat tema terang. Error: {e}")

    # Membuat dan menampilkan jendela utama
    window = MainWindow(config=config)
    window.show()

    sys.exit(app.exec())


# --- Titik masuk utama aplikasi (sekarang hanya memanggil run) ---
# Blok ini hanya akan berjalan jika Anda mengeksekusi file ini secara langsung
if __name__ == "__main__":
    run()