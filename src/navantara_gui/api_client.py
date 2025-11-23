# src/navantara_gui/api_client.py
import socketio
import numpy as np
import cv2
import base64  # <--- 1. TAMBAHAN PENTING

from PySide6.QtCore import QObject, Signal, Slot


class ApiClient(QObject):
    """
    Klien WebSocket yang berkomunikasi dengan backend. Mengadopsi pola "pull"
    di mana ia secara proaktif meminta stream data setelah terhubung.
    """

    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)
    # Sinyal untuk frame video (tetap menggunakan np.ndarray agar kompatibel dengan VideoView lama)
    frame_cam1_updated = Signal(np.ndarray)
    frame_cam2_updated = Signal(np.ndarray)

    def __init__(self, config):
        super().__init__()
        backend_config = config.get("backend_connection", {})
        self.base_url = f"http://{backend_config.get('ip_address', '127.0.0.1')}:{backend_config.get('port', 5000)}"

        # Kamus ini akan menyimpan state lengkap dan persisten di sisi GUI
        self.full_gui_state = {}

        # Inisialisasi klien Socket.IO
        self.sio = socketio.Client()
        self.setup_event_handlers()

    def connect_to_server(self):
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

        @self.sio.on("connect")
        def on_sio_connect():
            self.connection_status_changed.emit(True, "Terhubung ke Backend")
            print("Berhasil terhubung! Meminta stream data dari server...")
            self.sio.start_background_task(self.initial_stream_request)

        @self.sio.event
        def disconnect():
            # Reset state lengkap saat koneksi terputus
            self.full_gui_state = {}
            self.connection_status_changed.emit(False, "Koneksi terputus")
            print("Koneksi ke server terputus.")

        @self.sio.on("telemetry_update")
        def on_telemetry_update(data):
            # 'data' adalah 'delta_payload'
            try:
                self.full_gui_state.update(data)
                self.data_updated.emit(self.full_gui_state.copy())
            except Exception as e:
                print(f"[ApiClient] Gagal memproses telemetry_update: {e}")

        # --- [PERBAIKAN HANDLER VIDEO] ---
        @self.sio.on("frame_cam1")
        def on_frame_cam1(data):
            try:
                # 1. Cek tipe data (Bytes atau String/Base64)
                if not isinstance(data, (bytes, str)):
                    print(f"[API-CAM1] Tipe data salah: {type(data)}. Melompati.")
                    return

                # 2. Jika String (Base64), decode ke Bytes dulu
                if isinstance(data, str):
                    try:
                        data = base64.b64decode(data)
                    except Exception as e:
                        print(f"[API-CAM1] Gagal decode Base64: {e}")
                        return

                # 3. Decode Bytes JPEG ke OpenCV Image (numpy array)
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if frame is not None:
                    self.frame_cam1_updated.emit(frame)
                else:
                    print("[API-CAM1] Gagal decode frame (frame is None).")

            except Exception as e:
                print(f"[API-CAM1] Error processing frame: {e}")

        @self.sio.on("frame_cam2")
        def on_frame_cam2(data):
            try:
                # 1. Cek tipe data
                if not isinstance(data, (bytes, str)):
                    print(f"[API-CAM2] Tipe data salah: {type(data)}. Melompati.")
                    return

                # 2. Jika String (Base64), decode ke Bytes dulu
                if isinstance(data, str):
                    try:
                        data = base64.b64decode(data)
                    except Exception as e:
                        print(f"[API-CAM2] Gagal decode Base64: {e}")
                        return

                # 3. Decode Bytes JPEG ke OpenCV Image
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if frame is not None:
                    self.frame_cam2_updated.emit(frame)
                else:
                    print("[API-CAM2] Gagal decode frame (frame is None).")

            except Exception as e:
                print(f"[API-CAM2] Error processing frame: {e}")

        # --- [AKHIR PERBAIKAN] ---

    def initial_stream_request(self):
        """Fungsi yang dijalankan di latar belakang untuk meminta stream awal."""
        self.sio.sleep(0.1)
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
