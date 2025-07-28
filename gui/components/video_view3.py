# gui/components/video_view.py
# Komponen untuk menampilkan stream video dari kamera ASV.
# Versi ini menampilkan satu feed kamera utama.

import time
from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QLabel
from PySide6.QtGui import QPixmap, QImage, QPainter, QColor, QFont
from PySide6.QtCore import Qt, QTimer, Slot

class VideoView(QGroupBox):
    """
    Sebuah GroupBox yang menampilkan satu feed video utama.
    Untuk sekarang, ini akan mensimulasikan feed dengan menggambar frame palsu.
    """
    def __init__(self, title="Camera Feed"):
        super().__init__(title)

        # Label untuk menampilkan gambar/video
        self.video_label = QLabel("Connecting to camera...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: black; color: white;")
        self.video_label.setMinimumSize(640, 480) # Beri ukuran minimum
        
        # Layout utama
        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        self.setLayout(layout)

        # Timer untuk mengupdate frame video secara berkala
        self.timer = QTimer(self)
        self.timer.setInterval(100) # Update setiap 100ms (10 FPS)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start()

    @Slot()
    def update_frame(self):
        """Menggambar frame palsu dan menampilkannya di label."""
        # Ukuran frame
        width = self.video_label.width()
        height = self.video_label.height()

        if width <= 0 or height <= 0: return # Jangan gambar jika widget belum terlihat

        # Buat QImage kosong
        image = QImage(width, height, QImage.Format_RGB32)
        image.fill(Qt.black)

        # Gunakan QPainter untuk menggambar di atas gambar
        painter = QPainter(image)
        
        # Gambar kotak hijau sebagai penanda
        painter.setPen(QColor("lightgreen"))
        painter.drawRect(10, 10, width - 20, height - 20)

        # Gambar teks timestamp
        font = QFont()
        font.setPointSize(12)
        painter.setFont(font)
        timestamp = time.strftime('%H:%M:%S', time.localtime())
        painter.drawText(20, 40, f"LIVE FEED @ {timestamp}")
        painter.drawText(20, 70, "Status: SIMULATED")

        painter.end()

        # Set gambar yang sudah digambar ke label
        self.video_label.setPixmap(QPixmap.fromImage(image))

    def stop_feed(self):
        """Menghentikan timer saat tidak dibutuhkan."""
        self.timer.stop()

