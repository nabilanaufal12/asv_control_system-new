# gui/components/video_view.py
# --- FINAL: Menambahkan kembali kontrol kamera (Refresh, Invert, Pilih Kamera) ---

import cv2
import requests
import numpy as np
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QComboBox
from PySide6.QtCore import Signal, Slot, Qt, QThread
from PySide6.QtGui import QImage, QPixmap

class VideoStreamThread(QThread):
    """Thread untuk mengambil stream video dari URL backend secara terus-menerus."""
    frame_ready = Signal(np.ndarray)

    def __init__(self, stream_url, parent=None):
        super().__init__(parent)
        self.stream_url = stream_url
        self.running = False

    def run(self):
        """Mulai mengambil stream video dan memancarkan frame."""
        self.running = True
        try:
            with requests.get(self.stream_url, stream=True, timeout=5) as r:
                if r.status_code != 200:
                    print(f"Error: Gagal terhubung ke video stream (status: {r.status_code})")
                    return
                
                bytes_buffer = bytes()
                for chunk in r.iter_content(chunk_size=4096):
                    if not self.running:
                        break
                    
                    bytes_buffer += chunk
                    a = bytes_buffer.find(b'\xff\xd8')
                    b = bytes_buffer.find(b'\xff\xd9')
                    
                    if a != -1 and b != -1:
                        jpg = bytes_buffer[a:b+2]
                        bytes_buffer = bytes_buffer[b+2:]
                        if jpg:
                            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                            if frame is not None:
                                self.frame_ready.emit(frame)
        except requests.exceptions.RequestException as e:
            print(f"Koneksi ke video stream gagal: {e}")
        finally:
            print("Video stream thread dihentikan.")
            self.running = False

    def stop(self):
        """Menghentikan thread."""
        self.running = False

class VideoView(QWidget):
    """Widget utama yang menampilkan video dan kontrolnya."""
    def __init__(self, config, parent=None):
        super().__init__(parent)
        
        backend_config = config.get("backend_connection", {})
        ip = backend_config.get('ip_address', '127.0.0.1')
        port = backend_config.get('port', 5000)
        self.base_url = f"http://{ip}:{port}"
        stream_url = f"{self.base_url}/video_feed"

        # --- Elemen UI untuk Kontrol Kamera ---
        self.camera_selector = QComboBox()
        self.refresh_button = QPushButton("Refresh List")
        self.invert_button = QPushButton("Invert Logic")
        self.invert_button.setCheckable(True)

        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("Sumber Kamera:"))
        control_layout.addWidget(self.camera_selector, 1)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(self.invert_button)
        
        self.label = QLabel("Menghubungkan ke Video Stream...")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setMinimumSize(640, 480)
        self.label.setStyleSheet("background-color: #2c3e50; color: white; font-size: 16px;")

        main_layout = QVBoxLayout(self)
        main_layout.addLayout(control_layout)
        main_layout.addWidget(self.label, 1)

        self.stream_thread = VideoStreamThread(stream_url)
        self.stream_thread.frame_ready.connect(self.set_frame)
        self.stream_thread.start()
        
        # --- Hubungkan Tombol ke Fungsi API ---
        self.refresh_button.clicked.connect(self.refresh_camera_list)
        self.invert_button.clicked.connect(self.toggle_inversion)
        self.camera_selector.currentIndexChanged.connect(self.select_camera)
        
        self.refresh_camera_list() # Panggil sekali di awal untuk mengisi daftar

    def send_vision_command(self, command, payload=None):
        """Fungsi helper untuk mengirim perintah ke endpoint visi di backend."""
        try:
            requests.post(f"{self.base_url}/vision_command", json={"command": command, "payload": payload}, timeout=2)
        except requests.RequestException as e:
            print(f"Gagal mengirim perintah visi: {e}")

    def refresh_camera_list(self):
        """Meminta daftar kamera dari backend dan mengisinya ke ComboBox."""
        try:
            response = requests.get(f"{self.base_url}/list_cameras", timeout=3)
            if response.status_code == 200:
                self.camera_selector.blockSignals(True) # Cegah sinyal terpicu saat daftar diisi
                self.camera_selector.clear()
                cameras = response.json().get("cameras", [])
                if cameras:
                    self.camera_selector.addItems([f"Kamera {cam}" for cam in cameras])
                else:
                    self.camera_selector.addItem("Tidak ada kamera")
                self.camera_selector.blockSignals(False) # Aktifkan kembali sinyal
            else:
                print("Gagal mengambil daftar kamera dari backend.")
        except requests.RequestException as e:
            print(f"Gagal terhubung ke backend untuk refresh kamera: {e}")

    def toggle_inversion(self):
        is_checked = self.invert_button.isChecked()
        print(f"Mengirim perintah Invert Logic: {is_checked}")
        self.send_vision_command("SET_INVERT", is_checked)
        if is_checked:
            self.invert_button.setStyleSheet("background-color: #e74c3c; color: white;")
        else:
            self.invert_button.setStyleSheet("")
            
    def select_camera(self, index):
        """Mengirim indeks kamera yang dipilih ke backend."""
        if index < 0: return
        cam_text = self.camera_selector.itemText(index)
        try:
            cam_index = int(cam_text.split(" ")[1])
            print(f"Memilih kamera indeks: {cam_index}")
            self.send_vision_command("SELECT_CAMERA", cam_index)
        except (IndexError, ValueError):
            pass # Abaikan jika itemnya "Tidak ada kamera"

    @Slot(np.ndarray)
    def set_frame(self, frame):
        if frame is None: return
        try:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except Exception:
            pass # Abaikan error tampilan frame sesekali

    def stop_camera(self):
        """Metode untuk menghentikan thread video saat aplikasi ditutup."""
        if self.stream_thread.isRunning():
            self.stream_thread.stop()
            self.stream_thread.wait(2000)

    @Slot(str)
    def set_mode(self, mode):
        """Mengirim status mode (AUTO/MANUAL) ke backend."""
        is_auto = (mode == "AUTO")
        self.send_vision_command("SET_MODE", is_auto)