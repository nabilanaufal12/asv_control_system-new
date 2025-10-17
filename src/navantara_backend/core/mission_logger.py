# src/navantara_backend/core/mission_logger.py
import csv
import threading
from datetime import datetime
import os


class MissionLogger:
    def __init__(self, log_dir="mission_logs"):
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.telemetry_log_path = os.path.join(log_dir, f"telemetry_{timestamp}.csv")
        self.event_log_path = os.path.join(log_dir, f"events_{timestamp}.log")

        self._lock = threading.Lock()

        # Buka file dan tulis header untuk file CSV telemetri
        self.telemetry_file = open(self.telemetry_log_path, "w", newline="")
        self.telemetry_writer = csv.writer(self.telemetry_file)
        self.telemetry_header = [
            "timestamp",
            "latitude",
            "longitude",
            "heading",
            "speed",
            "battery_voltage",
            "control_mode",
            "status",
            "current_waypoint_index",
        ]
        self.telemetry_writer.writerow(self.telemetry_header)

    def log_telemetry(self, state_data):
        with self._lock:
            try:
                timestamp = datetime.now().isoformat()
                row = [timestamp] + [
                    state_data.get(key, "N/A") for key in self.telemetry_header[1:]
                ]
                self.telemetry_writer.writerow(row)
            except Exception as e:
                print(f"[Logger] Gagal menulis log telemetri: {e}")

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
