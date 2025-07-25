# gui/api_client.py
# VERSI DUMMY (ALWAYS CONNECTED): Disambungkan secara otomatis untuk pengujian yang lebih mudah.

import time
import random
import threading
from PySide6.QtCore import QObject, Signal

class ApiClient(QObject):
    """
    Kelas ini bertindak sebagai backend palsu.
    Ia akan langsung aktif dan menghasilkan data saat aplikasi dimulai.
    """
    data_updated = Signal(dict)
    connection_failed = Signal(str) 

    def __init__(self):
        super().__init__()
        
        self.is_connected = True 
        self.current_state = {
            "latitude": -6.2088, "longitude": 106.8456, "heading": 90.0,
            "speed": 0.0, "battery_voltage": 12.5, "status": "IDLE",
            "mission_time": "00:00:00", "control_mode": "MANUAL"
        }
        
        self._simulation_thread = threading.Thread(target=self._simulate_data_changes, daemon=True)
        self._simulation_thread.start()
        print("[ApiClient (Dummy)] Simulasi Backend langsung dimulai.")

    def configure_and_connect(self, details):
        """Fungsi ini sekarang hanya mencetak pesan, karena sudah otomatis terhubung."""
        print(f"[ApiClient (Dummy)] Pengaturan koneksi diterima: {details}")

    def disconnect(self):
        """Menghentikan simulasi."""
        self.is_connected = False
        print("[ApiClient (Dummy)] Simulasi Backend Dihentikan.")

    def _simulate_data_changes(self):
        """Fungsi yang berjalan di background untuk menghasilkan data palsu."""
        start_time = time.time()
        while self.is_connected:
            if self.current_state["control_mode"] == "AUTO":
                self.current_state["status"] = "NAVIGATING"
            else: # Mode MANUAL
                self.current_state["status"] = "IDLE"

            elapsed = time.time() - start_time
            self.current_state["mission_time"] = time.strftime('%H:%M:%S', time.gmtime(elapsed))
            self.data_updated.emit(self.current_state)
            time.sleep(1)

    def send_command(self, command, payload=None):
        """Mensimulasikan pengiriman perintah."""
        print(f"[ApiClient (Dummy)] Perintah diterima: {command}, Payload: {payload}")
        if not self.is_connected:
            return

        if command == "CHANGE_MODE":
            self.current_state["control_mode"] = payload
        elif command == "EMERGENCY_STOP":
            self.current_state["control_mode"] = "MANUAL"
        
        self.data_updated.emit(self.current_state)
