# gui/main.py
# File ini adalah titik masuk untuk menjalankan aplikasi GUI.

import sys
import os
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QFile, QTextStream

# --- PERBAIKAN PATH ---
# Menambahkan direktori root proyek ke sys.path agar semua modul dapat ditemukan.
# Ini adalah solusi yang lebih robust daripada mengubah impor di setiap file.
try:
    # Menentukan path ke direktori 'gui' tempat file ini berada
    gui_dir = os.path.dirname(os.path.abspath(__file__))
    # Naik satu level untuk mendapatkan root proyek
    project_root = os.path.dirname(gui_dir)
    # Menambahkan root proyek ke path Python
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
except NameError:
    # Fallback jika __file__ tidak terdefinisi (misal, di beberapa environment interaktif)
    sys.path.insert(0, '.')

# --- IMPOR SETELAH PATH DIPERBAIKI ---
# Sekarang kita bisa mengimpor dari 'gui' sebagai package level atas
from gui.views.main_window import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Muat dan terapkan stylesheet untuk tema gelap
    try:
        # Menggunakan path yang relatif terhadap direktori 'gui'
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

    window = MainWindow()
    window.show()

    sys.exit(app.exec())