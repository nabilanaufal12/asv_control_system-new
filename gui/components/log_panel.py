# gui/components/log_panel.py
# Komponen untuk menampilkan log data mentah dari backend.

from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QTextEdit
from PySide6.QtCore import Slot
from PySide6.QtGui import QFont

class LogPanel(QGroupBox):
    """
    Panel yang menampilkan log data mentah yang diterima dari backend.
    """
    def __init__(self, title="Raw Data Log"):
        super().__init__(title)

        main_layout = QVBoxLayout(self)

        # Buat area teks untuk menampilkan log
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True) # Agar tidak bisa diedit oleh pengguna
        
        # Atur font agar mudah dibaca
        font = QFont("Courier")
        font.setPointSize(10)
        self.log_display.setFont(font)
        
        main_layout.addWidget(self.log_display)

    @Slot(dict)
    def update_log(self, data):
        """
        Menerima data dictionary, mengubahnya menjadi string,
        dan menampilkannya di area log.
        """
        # Ubah dictionary menjadi string yang rapi
        log_text = str(data)
        
        # Tambahkan teks baru ke baris paling bawah
        self.log_display.append(log_text)
        
        # Auto-scroll ke bawah
        self.log_display.verticalScrollBar().setValue(self.log_display.verticalScrollBar().maximum())

