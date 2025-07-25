# gui/main.py
# File ini adalah titik masuk untuk menjalankan aplikasi GUI.

import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QFile, QTextStream
from views.main_window import MainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    # Muat dan terapkan stylesheet untuk tema gelap
    try:
        theme_file = QFile("gui/assets/resources/dark_theme.qss")
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
