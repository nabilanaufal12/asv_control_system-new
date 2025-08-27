# gui/components/log_panel.py
# --- VERSI FINAL: Dengan perbaikan "Smart Scrolling" ---

import json
from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QTextEdit
from PySide6.QtCore import Slot
from PySide6.QtGui import QFont


class LogPanel(QGroupBox):
    """
    Panel yang menampilkan log data mentah yang diterima dari backend.
    """

    def __init__(self, config, title="Raw Data Log"):
        super().__init__(title)

        self.config = config

        main_layout = QVBoxLayout(self)
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)

        font = QFont("Courier")
        font.setPointSize(10)
        self.log_display.setFont(font)

        main_layout.addWidget(self.log_display)
        self.setLayout(main_layout)

    @Slot(dict)
    def update_log(self, data):
        """
        Menerima data, menampilkannya, dan melakukan auto-scroll
        hanya jika pengguna sudah berada di paling bawah.
        """
        # --- PERBAIKAN DIMULAI DI SINI ---
        scrollbar = self.log_display.verticalScrollBar()
        # Cek apakah scrollbar ada di posisi paling bawah sebelum menambahkan teks baru
        # Diberi sedikit toleransi (10 piksel) untuk memastikan
        is_at_bottom = scrollbar.value() >= (scrollbar.maximum() - 10)
        
        try:
            log_text = json.dumps(data, indent=4)
            self.log_display.append(log_text)
            self.log_display.append("-" * 40)
        except Exception as e:
            self.log_display.append(str(data))
            print(f"LogPanel Error: Gagal mengubah data ke JSON. Error: {e}")

        # Hanya lakukan auto-scroll jika sebelumnya sudah di bawah
        if is_at_bottom:
            scrollbar.setValue(scrollbar.maximum())
        # --- AKHIR PERBAIKAN ---