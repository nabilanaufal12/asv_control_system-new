# src/navantara_backend/core/mission_logger.py
import csv
import threading
import os
from datetime import datetime


class MissionLogger:
    def __init__(self, race_id=1):
        self.telemetry_file = None
        self.telemetry_writer = None
        self.fieldnames = ["Day", "Date", "Time", "GPS", "SOG", "COG", "HDG"]
        self._lock = threading.Lock()

        # Mulai log untuk race default
        self.start_new_race_log(race_id)

    def start_new_race_log(self, race_id):
        with self._lock:
            if self.telemetry_file:
                self.telemetry_file.close()

            log_dir = os.path.join(os.getcwd(), "logs", "captures", f"race_{race_id}")
            if not os.path.exists(log_dir):
                os.makedirs(log_dir, exist_ok=True)

            self.telemetry_log_path = os.path.join(log_dir, "telemetry.csv")
            self.event_log_path = os.path.join(log_dir, "events.log")

            # Periksa apakah file sudah ada dan ukurannya > 0
            file_exists = (
                os.path.exists(self.telemetry_log_path)
                and os.path.getsize(self.telemetry_log_path) > 0
            )

            # Buka dengan mode append ("a")
            self.telemetry_file = open(
                self.telemetry_log_path, "a", newline="", buffering=1
            )
            self.telemetry_writer = csv.DictWriter(
                self.telemetry_file, fieldnames=self.fieldnames
            )

            # Tulis header HANYA JIKA file baru/kosong
            if not file_exists:
                self.telemetry_writer.writeheader()

            print(
                f"[Logger] Started CSV log for race_{race_id} at {self.telemetry_log_path}"
            )

    def log_telemetry(self, state_data):
        """
        Menerima state_data (dict) dan memformatnya menjadi kolom Day, Date, dll.
        """
        with self._lock:
            try:
                if not self.telemetry_file or not self.telemetry_writer:
                    return

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

    # --- DUMMY FUNCTIONS ---
    # Agar jika dipanggil oleh kode GUI lama tidak terjadi error
    def start_csv_log(self, headers=None):
        pass

    def stop_csv_log(self):
        pass
