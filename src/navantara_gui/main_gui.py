# src/navantara_gui/main_gui.py
# --- VERSI FINAL: FIX E402 ---

import sys
import os
import json
import warnings

# [FIX] Pindahkan import PySide6 ke atas (sebelum kode warnings)
from PySide6.QtWidgets import QApplication

# Mengabaikan FutureWarning yang tidak relevan dari library lain
warnings.simplefilter(action="ignore", category=FutureWarning)

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
# Gunakan # noqa: E402 karena import ini WAJIB berada setelah manipulasi sys.path
from navantara_gui.views.main_window import MainWindow  # noqa: E402


# --- FUNGSI BARU UNTUK MENJALANKAN APLIKASI ---
def run():
    """
    Inisialisasi dan menjalankan seluruh aplikasi GUI.
    """
    app = QApplication(sys.argv)

    config = {}
    try:
        # Menemukan path root proyek secara dinamis dari lokasi file ini
        gui_dir = os.path.dirname(os.path.abspath(__file__))
        # Dari 'src/navantara_gui', kita naik DUA tingkat untuk mencapai root proyek
        project_root = os.path.dirname(os.path.dirname(gui_dir))
        config_path = os.path.join(project_root, "config", "config.json")

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


# --- Titik masuk utama aplikasi ---
if __name__ == "__main__":
    run()
