# gui/main.py
# --- VERSI FINAL: Titik masuk tunggal untuk aplikasi All-in-One ---

import sys
import os
import json
import warnings

# Mengabaikan FutureWarning yang tidak relevan dari library lain
warnings.simplefilter(action="ignore", category=FutureWarning)

from PySide6.QtWidgets import QApplication

# --- Blok ini memperbaiki path agar impor dari folder lain berhasil ---
try:
    gui_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(gui_dir)
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
except NameError:
    # Fallback jika __file__ tidak terdefinisi
    sys.path.insert(0, ".")

# --- Impor setelah path diperbaiki ---
from gui.views.main_window import MainWindow

# --- Titik masuk utama aplikasi ---
if __name__ == "__main__":
    app = QApplication(sys.argv)

    config = {}
    try:
        # Perbaikan: gunakan project_root yang sudah didefinisikan
        config_path = os.path.join(project_root, "config.json")
        with open(config_path, "r") as f:
            config = json.load(f)
        print("File konfigurasi 'config.json' berhasil dimuat.")
    except Exception as e:
        print(f"KRITIS: Gagal memuat 'config.json'. Aplikasi akan berhenti. Error: {e}")
        sys.exit(1)  # Keluar jika config tidak bisa dimuat

    # Memuat dan menerapkan stylesheet untuk tema gelap (opsional)
    try:
        # Perbaikan: gunakan gui_dir yang sudah didefinisikan
        theme_path = os.path.join(gui_dir, "assets", "resources", "dark_theme.qss")
        with open(theme_path, "r") as f:
            app.setStyleSheet(f.read())
        print("Tema gelap berhasil dimuat.")
    except Exception as e:
        print(f"Peringatan: Gagal memuat tema gelap. Error: {e}")

    # --- PERUBAHAN UTAMA: Cukup buat instance MainWindow ---
    # MainWindow sekarang bertanggung jawab untuk menginisialisasi semua logika backend.
    window = MainWindow(config=config)
    window.show()

    # Menjalankan aplikasi Qt
    sys.exit(app.exec())
