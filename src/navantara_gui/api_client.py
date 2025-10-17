# src/navantara_gui/api_client.py
import socketio
import numpy as np
import cv2

from PySide6.QtCore import QObject, Signal, Slot


class ApiClient(QObject):
    """
    Klien WebSocket yang berkomunikasi dengan backend. Mengadopsi pola "pull"
    di mana ia secara proaktif meminta stream data setelah terhubung.
    """

    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)
    # Sinyal baru untuk frame video yang diterima dan sudah di-decode
    frame_cam1_updated = Signal(np.ndarray)
    frame_cam2_updated = Signal(np.ndarray)

    def __init__(self, config):
        super().__init__()
        backend_config = config.get("backend_connection", {})
        self.base_url = f"http://{backend_config.get('ip_address', '127.0.0.1')}:{backend_config.get('port', 5000)}"

        # Inisialisasi klien Socket.IO
        self.sio = socketio.Client()
        self.setup_event_handlers()

    def connect(self):
        """
        Memulai koneksi ke server. Pustaka menangani koneksi non-blocking
        secara internal, sehingga tidak perlu thread manual.
        """
        print(f"ApiClient mencoba terhubung ke server WebSocket di {self.base_url}")
        try:
            self.sio.connect(self.base_url, transports=["websocket"])
        except socketio.exceptions.ConnectionError as e:
            print(f"Koneksi ke server WebSocket gagal: {e}")
            self.connection_status_changed.emit(False, "Backend tidak terjangkau")

    def setup_event_handlers(self):
        """Mendefinisikan callback untuk event yang diterima dari server."""

        @self.sio.event
        def connect():
            self.connection_status_changed.emit(True, "Terhubung ke Backend")
            print("Berhasil terhubung! Meminta stream data dari server...")
            # --- PERBAIKAN: Gunakan background task untuk menghindari race condition ---
            # Ini memastikan permintaan dikirim setelah event 'connect' selesai sepenuhnya.
            self.sio.start_background_task(self.initial_stream_request)

        @self.sio.event
        def disconnect():
            self.connection_status_changed.emit(False, "Koneksi terputus")
            print("Koneksi ke server terputus.")

        @self.sio.on("telemetry_update")
        def on_telemetry_update(data):
            self.data_updated.emit(data)

        @self.sio.on("frame_cam1")
        def on_frame_cam1(data):
            # Ubah data byte JPEG kembali menjadi gambar OpenCV
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                # Kirim frame sebagai sinyal Qt untuk ditampilkan oleh UI
                self.frame_cam1_updated.emit(frame)

        @self.sio.on("frame_cam2")
        def on_frame_cam2(data):
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.frame_cam2_updated.emit(frame)

    def initial_stream_request(self):
        """Fungsi yang dijalankan di latar belakang untuk meminta stream awal."""
        self.sio.sleep(0.1)  # Beri sedikit jeda agar state koneksi stabil
        self.request_data_stream(True)

    @Slot(bool)
    def request_data_stream(self, start: bool):
        """Mengirim event untuk memulai atau menghentikan stream data dari server."""
        if self.sio.connected:
            self.sio.emit("request_stream", {"status": start})
            print(
                f"Mengirim permintaan untuk {'memulai' if start else 'menghentikan'} stream."
            )
        else:
            print("Tidak bisa meminta stream, belum terhubung ke server.")

    def send_command(self, command_name, payload_data=None):
        """Fungsi helper terpusat untuk mengirim semua perintah ke backend."""
        if payload_data is None:
            payload_data = {}
        if self.sio.connected:
            self.sio.emit("command", {"command": command_name, "payload": payload_data})
        else:
            print(f"Gagal mengirim perintah '{command_name}', tidak terhubung.")

    def shutdown(self):
        """Memutuskan koneksi dengan bersih saat aplikasi ditutup."""
        print("Memutuskan koneksi WebSocket...")
        if self.sio.connected:
            self.request_data_stream(False)
            self.sio.disconnect()
