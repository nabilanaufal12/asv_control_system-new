# src/navantara_backend/core/mission_logger.py
import csv
import threading
import os
import logging
from datetime import datetime


class MissionLogger:
    def __init__(self, base_log_dir=None):
        if base_log_dir is None:
            # Otomatis gunakan folder 'logs' di root project
            project_root = os.path.dirname(
                os.path.dirname(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                )
            )
            self.base_log_dir = os.path.join(project_root, "logs")
        else:
            self.base_log_dir = os.path.abspath(base_log_dir)

        os.makedirs(self.base_log_dir, exist_ok=True)

        self._lock = threading.Lock()
        self.telemetry_file = None
        self.telemetry_writer = None
        self.fieldnames = ["Day", "Date", "Time", "GPS", "SOG", "COG", "HDG"]
        self.current_race_id = 0
        self.race_dir = self.base_log_dir
        self.telemetry_dir = self.base_log_dir
        self.captures_dir = os.path.join(self.base_log_dir, "captures")
        self.event_log_path = os.path.join(self.base_log_dir, "events.log")
        self._consecutive_log_errors = 0

        self.start_new_race()

    def start_new_race(self):
        with self._lock:
            try:
                # Tutup file sebelumnya jika masih terbuka
                if self.telemetry_file:
                    try:
                        if not self.telemetry_file.closed:
                            self.telemetry_file.close()
                    except Exception:
                        pass
                self.telemetry_file = None
                self.telemetry_writer = None

                os.makedirs(self.base_log_dir, exist_ok=True)

                # Cari Race ID tertinggi saat ini
                max_id = 0
                for entry in os.listdir(self.base_log_dir):
                    if entry.startswith("race_") and os.path.isdir(
                        os.path.join(self.base_log_dir, entry)
                    ):
                        try:
                            num = int(entry.split("_")[1])
                            if num > max_id:
                                max_id = num
                        except ValueError:
                            continue

                # Buat sesi race baru (increment)
                self.current_race_id = max_id + 1

                self.race_dir = os.path.join(
                    self.base_log_dir, f"race_{self.current_race_id}"
                )
                self.telemetry_dir = os.path.join(self.race_dir, "telemetry")
                self.captures_dir = os.path.join(self.race_dir, "captures")

                os.makedirs(self.telemetry_dir, exist_ok=True)
                os.makedirs(self.captures_dir, exist_ok=True)

                self.telemetry_log_path = os.path.join(
                    self.telemetry_dir, "mission_data.csv"
                )
                self.event_log_path = os.path.join(self.race_dir, "global_events.log")

                # Buka CSV baru untuk race ini
                self.telemetry_file = open(self.telemetry_log_path, "a", newline="")
                self.telemetry_writer = csv.DictWriter(
                    self.telemetry_file, fieldnames=self.fieldnames
                )

                # Jika file masih kosong, tulis header
                if os.path.getsize(self.telemetry_log_path) == 0:
                    self.telemetry_writer.writeheader()

                self._consecutive_log_errors = 0
                print(f"[Logger] Started New Race: {self.current_race_id}")
            except Exception as e:
                logging.error(f"[Logger] Gagal memulai race baru: {e}")
                self.telemetry_file = None
                self.telemetry_writer = None

    def log_telemetry(self, state_data):
        """
        Menerima state_data (dict) dan memformatnya menjadi kolom Day, Date, dll.
        """
        with self._lock:
            if (
                not self.telemetry_file
                or self.telemetry_file.closed
                or not self.telemetry_writer
            ):
                return
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
                self.telemetry_file.flush()  # Pastikan ditulis ke disk
                self._consecutive_log_errors = 0

            except Exception as e:
                self._consecutive_log_errors += 1
                if self._consecutive_log_errors <= 3:
                    logging.warning(f"[Logger] Gagal menulis log: {e}")

    def log_event(self, message):
        with self._lock:
            try:
                os.makedirs(os.path.dirname(self.event_log_path), exist_ok=True)
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                with open(self.event_log_path, "a") as f:
                    f.write(f"[{timestamp}] {message}\n")
            except Exception:
                pass

    def get_current_capture_dir(self):
        with self._lock:
            return self.captures_dir

    def stop(self):
        with self._lock:
            if self.telemetry_file:
                try:
                    if not self.telemetry_file.closed:
                        self.telemetry_file.close()
                except Exception:
                    pass
                self.telemetry_file = None
                self.telemetry_writer = None
                print("[Logger] File log ditutup.")
