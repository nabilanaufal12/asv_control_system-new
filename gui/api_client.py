# gui/api_client.py
# --- VERSI MIGRASI: Menjadi HTTP Client Murni ---

import time
import threading
import requests
from PySide6.QtCore import QObject, Signal, Slot

class ApiClient(QObject):
    """
    Kelas ini sekarang bertindak sebagai jembatan antara GUI dan Backend Server.
    Semua komunikasi dilakukan via HTTP ke server (yang berjalan di PC yang sama atau di Jetson).
    """
    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)

    def __init__(self, config):
        super().__init__()
        self.config = config
        self.running = True
        
        # Ambil alamat IP backend dari config.json, default ke localhost ('127.0.0.1')
        backend_config = self.config.get("backend_connection", {})
        self.base_url = f"http://{backend_config.get('ip_address', '127.0.0.1')}:{backend_config.get('port', 5000)}"
        
        # State hanya digunakan untuk perbandingan, sumber kebenaran ada di backend
        self.current_state = {}

        # Thread untuk secara terus-menerus meminta status terbaru dari backend
        self._poll_thread = threading.Thread(target=self._poll_status_loop, daemon=True)
        self._poll_thread.start()
        print(f"ApiClient berjalan dalam MODE HTTP CLIENT, menargetkan backend di {self.base_url}")

    def _send_command(self, endpoint, payload):
        """Fungsi helper untuk mengirim perintah POST ke backend."""
        try:
            url = f"{self.base_url}/{endpoint}"
            response = requests.post(url, json=payload, timeout=2) # Timeout 2 detik
            if response.status_code == 200:
                print(f"Perintah '{payload.get('command')}' berhasil dikirim.")
                return response.json()
            else:
                print(f"Gagal mengirim perintah: {response.status_code}")
                self.connection_status_changed.emit(False, f"Backend Error: {response.status_code}")
        except requests.RequestException:
            # Error koneksi (mis. backend tidak berjalan)
            self.connection_status_changed.emit(False, "Backend tidak terjangkau")
        return None

    def _poll_status_loop(self):
        """Secara periodik (setiap 0.5 detik) meminta state terbaru dari backend."""
        while self.running:
            try:
                response = requests.get(f"{self.base_url}/status", timeout=2)
                if response.status_code == 200:
                    new_state = response.json()
                    
                    # --- PERBAIKAN DI SINI ---
                    # Selalu emit data_updated agar thread kamera dan komponen lain
                    # selalu mendapat data telemetri terbaru, bahkan jika kapal diam.
                    self.current_state = new_state
                    self.data_updated.emit(self.current_state)
                    # -------------------------
                    
                    # Tampilkan status koneksi berdasarkan data dari backend
                    is_serial_connected = new_state.get('is_connected_to_serial', False)
                    status_message = "Backend & Serial Terhubung" if is_serial_connected else "Backend OK (Serial Disconnected)"
                    self.connection_status_changed.emit(True, status_message)
                else:
                    self.connection_status_changed.emit(False, f"Backend Error ({response.status_code})")
            except requests.RequestException:
                self.connection_status_changed.emit(False, "Menunggu koneksi Backend...")
            
            time.sleep(0.5) # Interval polling status

    # --- KUMPULAN SLOT UNTUK MENANGANI SINYAL DARI GUI ---

    @Slot(dict)
    def connect_to_port(self, connection_details):
        """Mengirim permintaan ke backend untuk terhubung ke port serial."""
        print(f"GUI meminta koneksi serial: {connection_details}")
        payload = {"command": "CONFIGURE_SERIAL", "payload": connection_details}
        self._send_command("command", payload)

    @Slot(str)
    def handle_mode_change(self, mode):
        """Mengirim perubahan mode (MANUAL/AUTO) ke backend."""
        payload = {"command": "CHANGE_MODE", "payload": mode}
        self._send_command("command", payload)

    @Slot(list)
    def handle_manual_keys(self, keys):
        """Mengirim tombol keyboard yang sedang ditekan ke backend untuk kontrol manual."""
        payload = {"command": "MANUAL_CONTROL", "payload": keys}
        self._send_command("command", payload)

    @Slot(list)
    def set_waypoints(self, waypoints):
        """Mengirim daftar waypoints baru ke backend."""
        payload = {"command": "SET_WAYPOINTS", "payload": waypoints}
        self._send_command("command", payload)

    @Slot(dict)
    def handle_vision_status(self, vision_data):
        """Mengirim status dari modul computer vision ke backend untuk override."""
        payload = {"command": "VISION_OVERRIDE", "payload": vision_data}
        self._send_command("command", payload)

    def shutdown(self):
        """Memastikan thread polling berhenti saat aplikasi ditutup."""
        print("Memulai prosedur shutdown ApiClient...")
        self.running = False
        if self._poll_thread.is_alive():
            self._poll_thread.join(timeout=1.0)
        print("ApiClient shutdown selesai.")