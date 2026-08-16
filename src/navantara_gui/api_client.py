# src/navantara_gui/api_client.py
import socketio
import numpy as np
import cv2
import base64  # <--- 1. TAMBAHAN PENTING

from PySide6.QtCore import QObject, Signal, Slot, QThread

import urllib.request

# --- [OPTIMASI KEY MINIFICATION: MAPPING DICTIONARY] ---
# Untuk decode JSON key pendek kembali ke full key asli AsvState
REVERSE_KEY_MAP = {
    "lat": "latitude",
    "lon": "longitude",
    "hdg": "heading",
    "cog": "cog",
    "sog": "speed",
    "sts": "status",
    "mode": "control_mode",
    "ar": "active_arena",
    "wps": "waypoints",
    "cur_wp": "current_waypoint_index",
    "wp_idx": "nav_target_wp_index",
    "wp_tot": "nav_esp_total_wp",
    "wp_dst": "nav_dist_to_wp",
    "xte": "nav_xte_m",
    "err_hdg": "nav_heading_error",
    "tgt_brg": "nav_target_bearing",
    "sat": "nav_gps_sats",
    "srv": "nav_servo_cmd",
    "mot": "nav_motor_cmd",
    "m_srv": "manual_servo_cmd",
    "m_mot": "manual_motor_cmd",
    "time": "mission_time",
    "rc": "rc_channels",
    "conn": "is_connected_to_serial",
    "dbg_cnt": "debug_waypoint_counter",
    "vis": "vision_target",
    "esp_sts": "esp_status",
    "dk_st": "docking_state",
}


class MjpegStreamThread(QThread):
    frame_ready = Signal(bytes)

    def __init__(self, url, parent=None):
        super().__init__(parent)
        self.url = url
        self.running = False

    def run(self):
        self.running = True
        try:
            req = urllib.request.Request(self.url)
            with urllib.request.urlopen(req, timeout=5) as res:
                bytes_data = b''
                while self.running:
                    chunk = res.read(1024)
                    if not chunk:
                        break
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b+2]
                        bytes_data = bytes_data[b+2:]
                        self.frame_ready.emit(jpg)
        except Exception as e:
            print(f"[MjpegStreamThread] Error streaming from {self.url}: {e}")
        finally:
            self.running = False

    def request_stop(self):
        """Non-blocking: hanya set flag, tidak menunggu thread selesai.
        Aman dipanggil dari dalam event handler Socket.IO."""
        self.running = False

    def stop(self):
        """Blocking: set flag DAN tunggu thread selesai.
        Hanya dipanggil saat shutdown aplikasi (bukan saat disconnect)."""
        self.running = False
        self.wait(3000)  # Timeout 3 detik agar tidak hang selamanya

class ApiClient(QObject):
    """
    Klien WebSocket yang berkomunikasi dengan backend. Mengadopsi pola "pull"
    di mana ia secara proaktif meminta stream data setelah terhubung.
    """

    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)
    # Sinyal untuk frame video (sekarang menggunakan bytes dari MJPEG) agar kompatibel dengan VideoView lama)
    frame_cam1_updated = Signal(bytes)
    frame_cam2_updated = Signal(bytes)

    # Sinyal untuk sinkronisasi waypoint dua arah
    sync_waypoints_received = Signal(list)

    def __init__(self, config):
        super().__init__()
        backend_config = config.get("backend_connection", {})
        self.base_url = f"http://{backend_config.get('ip_address', '127.0.0.1')}:{backend_config.get('port', 5000)}"

        # Kamus ini akan menyimpan state lengkap dan persisten di sisi GUI
        self.full_gui_state = {}

        # Inisialisasi thread video
        self.cam1_thread = None
        self.cam2_thread = None

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
            # Mulai thread video (non-blocking, tidak menghambat event handler ini)
            self._start_video_threads()

        @self.sio.event
        def disconnect():
            # [FIX] Gunakan request_stop() (non-blocking) bukan stop() (blocking)
            # Stop yang blocking di sini menyebabkan deadlock dan crash 'QThread: Destroyed while thread is still running'.
            if self.cam1_thread:
                self.cam1_thread.request_stop()
            if self.cam2_thread:
                self.cam2_thread.request_stop()

            # Reset state lengkap saat koneksi terputus
            self.full_gui_state = {}
            self.connection_status_changed.emit(False, "Koneksi terputus")
            print("Koneksi ke server terputus.")

        @self.sio.on("telemetry_update")
        def on_telemetry_update(data):
            # 'data' adalah 'delta_payload' yang berisi key pendek
            try:
                # --- [REHYDRATE KEYS] ---
                full_keys_data = {}
                for key, value in data.items():
                    long_key = REVERSE_KEY_MAP.get(key, key)
                    full_keys_data[long_key] = value

                self.full_gui_state.update(full_keys_data)
                self.data_updated.emit(self.full_gui_state.copy())
            except Exception as e:
                print(f"[ApiClient] Gagal memproses telemetry_update: {e}")

        @self.sio.on("sync_waypoints")
        def on_sync_waypoints(data):
            try:
                self.sync_waypoints_received.emit(data)
                print(
                    f"[ApiClient] Menerima sync_waypoints dari ESP32 dengan {len(data)} titik."
                )
            except Exception as e:
                print(f"[ApiClient] Gagal memproses sync_waypoints: {e}")

    def _start_video_threads(self):
        """Memulai (atau me-restart) kedua thread streaming MJPEG."""
        # Hentikan thread lama secara non-blocking jika masih berjalan
        if self.cam1_thread is not None:
            self.cam1_thread.request_stop()
        if self.cam2_thread is not None:
            self.cam2_thread.request_stop()

        # Buat dan mulai thread baru
        self.cam1_thread = MjpegStreamThread(f"{self.base_url}/video_feed_1")
        self.cam1_thread.frame_ready.connect(self.frame_cam1_updated.emit)
        self.cam1_thread.start()

        self.cam2_thread = MjpegStreamThread(f"{self.base_url}/video_feed_2")
        self.cam2_thread.frame_ready.connect(self.frame_cam2_updated.emit)
        self.cam2_thread.start()

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
        try:
            self.request_data_stream(False)
        except Exception:
            pass
        self.sio.disconnect()
        # Saat shutdown aplikasi, pakai stop() yang blocking agar thread selesai sepenuhnya
        if self.cam1_thread:
            self.cam1_thread.stop()
        if self.cam2_thread:
            self.cam2_thread.stop()
