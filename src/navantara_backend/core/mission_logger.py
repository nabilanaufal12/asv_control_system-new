# src/navantara_backend/core/mission_logger.py
import csv
import threading
import os
from datetime import datetime


class MissionLogger:
    def __init__(self, log_dir="mission_logs"):
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # Nama file menggunakan timestamp saat start
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.telemetry_log_path = os.path.join(log_dir, f"mission_data_{timestamp}.csv")
        self.event_log_path = os.path.join(log_dir, f"events_{timestamp}.log")

        self._lock = threading.Lock()

        # --- SETUP CSV DENGAN FORMAT SESUAI PERMINTAAN ---
        self.telemetry_file = open(self.telemetry_log_path, "w", newline="")

        # Header kolom yang Anda minta
        self.fieldnames = ["Day", "Date", "Time", "GPS", "SOG", "COG", "HDG"]

        # Menggunakan DictWriter agar penulisan lebih rapi dan aman
        self.telemetry_writer = csv.DictWriter(
            self.telemetry_file, fieldnames=self.fieldnames
        )
        self.telemetry_writer.writeheader()

    def log_telemetry(self, state_data):
        """
        Menerima state_data (dict) dan memformatnya menjadi kolom Day, Date, dll.
        """
        with self._lock:
            try:
                now = datetime.now()

                # Ambil data dari state (dengan nilai default 0 jika error/kosong)
                lat = state_data.get("latitude", 0)
                lon = state_data.get("longitude", 0)
                speed = state_data.get("speed", 0)
                cog = state_data.get("cog", 0)
                heading = state_data.get("heading", 0)

                # Format data sesuai kolom
                row_payload = {
                    "Day": now.strftime("%A"),  # Nama Hari
                    "Date": now.strftime("%Y-%m-%d"),  # Tanggal
                    "Time": now.strftime("%H:%M:%S"),  # Jam
                    "GPS": f"{lat:.6f}, {lon:.6f}",  # Gabung Lat, Lon
                    "SOG": f"{speed:.2f}",  # Speed Over Ground
                    "COG": f"{cog:.1f}",  # Course Over Ground
                    "HDG": f"{heading:.1f}",  # Heading
                }

                # Tulis ke file
                self.telemetry_writer.writerow(row_payload)

            except Exception as e:
                print(f"[Logger] Gagal menulis log: {e}")

    def log_event(self, message):
        with self._lock:
            try:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                with open(self.event_log_path, "a") as f:
                    f.write(f"[{timestamp}] {message}\n")
            except Exception as e:
                print(f"[Logger] Gagal menulis log event: {e}")

    def stop(self):
        with self._lock:
            if self.telemetry_file:
                self.telemetry_file.close()
                self.telemetry_file = None
                print("[Logger] File log ditutup.")
