# gui/components/video_view.py
# --- VERSI FINAL: Disederhanakan dengan menghapus pilihan kamera manual ---

import base64
import socketio
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QSizePolicy
from PySide6.QtCore import Signal, Slot, Qt, QThread, QObject
from PySide6.QtGui import QImage, QPixmap

class KlienSocketIO(QObject):
    """
    Worker yang berjalan di thread terpisah untuk menangani komunikasi Socket.IO.
    """
    frame_baru = Signal(str)
    status_koneksi = Signal(str)

    def __init__(self, url_server):
        super().__init__()
        self.url_server = url_server
        self.sio = socketio.Client(reconnection_attempts=5, reconnection_delay=1)
        self.setup_event_handler()

    def setup_event_handler(self):
        """Mendefinisikan handler untuk event dari Socket.IO."""
        @self.sio.event
        def connect():
            self.status_koneksi.emit("Terhubung")
            print("GUI: Berhasil terhubung ke server WebSocket!")

        @self.sio.on('video_frame')
        def on_video_frame(data):
            self.frame_baru.emit(data)

        @self.sio.event
        def connect_error(data):
            self.status_koneksi.emit("Koneksi Gagal")
            print(f"GUI: Gagal terhubung ke server: {data}")

        @self.sio.event
        def disconnect():
            self.status_koneksi.emit("Terputus")
            print("GUI: Terputus dari server.")

    def jalankan(self):
        """Menghubungkan ke server dan menunggu event."""
        try:
            self.sio.connect(self.url_server, transports=['websocket'])
            self.sio.wait()
        except socketio.exceptions.ConnectionError as e:
            self.status_koneksi.emit("Error")
            print(f"GUI: Terjadi kesalahan koneksi: {e}")

    def kirim_perintah(self, perintah, payload=None):
        """Mengirim perintah ke backend melalui WebSocket."""
        if self.sio.connected:
            self.sio.emit('command', {'command': perintah, 'payload': payload})
        else:
            print(f"GUI: Tidak dapat mengirim perintah '{perintah}', koneksi tidak ada.")

    def berhenti(self):
        """Memutuskan koneksi klien."""
        if self.sio.connected:
            self.sio.disconnect()


class VideoView(QWidget):
    """Widget utama yang menampilkan feed video dan kontrolnya."""
    def __init__(self, config, parent=None):
        super().__init__(parent)
        
        config_backend = config.get("backend_connection", {})
        ip = config_backend.get('ip_address', '127.0.0.1')
        port = config_backend.get('port', 5000)
        url_server = f"http://{ip}:{port}"

        # --- Pengaturan Tampilan (UI) yang Disederhanakan ---
        self.tombol_invert = QPushButton("Invert Logic")
        self.tombol_invert.setCheckable(True)

        layout_kontrol = QHBoxLayout()
        layout_kontrol.addStretch() # Mendorong tombol ke kanan
        layout_kontrol.addWidget(self.tombol_invert)
        
        self.label_video = QLabel("Menghubungkan ke Stream Video...")
        self.label_video.setAlignment(Qt.AlignCenter)
        self.label_video.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.label_video.setMinimumSize(640, 480)
        self.label_video.setStyleSheet("background-color: black; color: white; font-size: 16px;")

        layout_utama = QVBoxLayout(self)
        layout_utama.addLayout(layout_kontrol)
        layout_utama.addWidget(self.label_video, 1)

        # --- Pengaturan Thread WebSocket ---
        self.thread_socket = QThread()
        self.klien_socket = KlienSocketIO(url_server)
        self.klien_socket.moveToThread(self.thread_socket)

        # Hubungkan sinyal dari worker ke slot
        self.thread_socket.started.connect(self.klien_socket.jalankan)
        self.klien_socket.frame_baru.connect(self.update_frame_video)
        self.klien_socket.status_koneksi.connect(self.update_status_koneksi)
        
        # Hubungkan sinyal dari elemen UI
        self.tombol_invert.clicked.connect(self.toggle_inversi)

        # Jalankan thread
        self.thread_socket.start()

    @Slot(str)
    def update_frame_video(self, string_base64):
        """Mendekode string base64 dan memperbarui label video dengan frame baru."""
        try:
            data_gambar = base64.b64decode(string_base64)
            gambar = QImage()
            gambar.loadFromData(data_gambar, "JPG")
            pixmap = QPixmap.fromImage(gambar)
            self.label_video.setPixmap(pixmap.scaled(self.label_video.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except Exception as e:
            print(f"GUI Error: Gagal memproses frame video: {e}")
            
    @Slot(str)
    def update_status_koneksi(self, status):
        """Memperbarui teks label berdasarkan status koneksi."""
        # Hanya tampilkan status jika tidak sedang menampilkan video
        if status != "Terhubung":
            # Periksa apakah label sedang menampilkan gambar atau tidak
            if not self.label_video.pixmap() or self.label_video.pixmap().isNull():
                 self.label_video.setText(f"Status Koneksi: {status}")

    def toggle_inversi(self):
        """Mengirim status tombol 'Invert' ke backend."""
        tercentang = self.tombol_invert.isChecked()
        self.klien_socket.kirim_perintah("SET_INVERT", tercentang)
        self.tombol_invert.setStyleSheet("background-color: #e74c3c; color: white;" if tercentang else "")
            
    @Slot(str)
    def set_mode(self, mode):
        """Mengirim mode ASV (AUTO/MANUAL) ke backend."""
        self.klien_socket.kirim_perintah("SET_MODE", mode)

    def closeEvent(self, event):
        """Menghentikan thread WebSocket dengan bersih saat widget ditutup."""
        print("GUI: Menutup VideoView, menghentikan klien WebSocket...")
        self.klien_socket.berhenti()
        self.thread_socket.quit()
        self.thread_socket.wait(2000)
        super().closeEvent(event)
