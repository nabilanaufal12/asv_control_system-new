# gui/components/video_view.py
# --- MODIFIKASI FINAL: Otomatis memilih kamera pertama yang aktif saat startup ---

import cv2
import requests
import numpy as np
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QComboBox
from PySide6.QtCore import Signal, Slot, Qt, QThread
from PySide6.QtGui import QImage, QPixmap

class VideoStreamThread(QThread):
    """Thread untuk mengambil stream video dari URL backend secara terus-menerus."""
    frame_ready = Signal(np.ndarray)
    connection_error = Signal(str)

    def __init__(self, stream_url, parent=None):
        super().__init__(parent)
        self.stream_url = stream_url
        self.running = False

    def run(self):
        """Mulai mengambil stream video dan memancarkan frame."""
        self.running = True
        try:
            with requests.get(self.stream_url, stream=True, timeout=(5, 10)) as r:
                r.raise_for_status()
                bytes_buffer = bytes()
                for chunk in r.iter_content(chunk_size=8192):
                    if not self.running:
                        break
                    
                    bytes_buffer += chunk
                    a = bytes_buffer.find(b'\\xff\\xd8')
                    b = bytes_buffer.find(b'\\xff\\xd9')
                    
                    if a != -1 and b != -1:
                        jpg = bytes_buffer[a:b+2]
                        bytes_buffer = bytes_buffer[b+2:]
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is not None:
                            self.frame_ready.emit(frame)
        except requests.exceptions.RequestException as e:
            error_message = f"Koneksi video stream gagal.\\nPastikan backend berjalan di {self.stream_url.split('/video_feed')[0]}\\n"
            self.connection_error.emit(error_message)
        finally:
            self.running = False

    def stop(self):
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
        self.stream_thread.connection_error.connect(self.show_error_message)
        self.stream_thread.start()
        
        self.refresh_button.clicked.connect(self.refresh_camera_list)
        self.invert_button.clicked.connect(self.toggle_inversion)
        self.camera_selector.currentIndexChanged.connect(self.select_camera_from_dropdown)
        
        # --- PERUBAHAN UTAMA: Panggil fungsi untuk otomatisasi di sini ---
        self.find_and_select_camera()

    def send_vision_command(self, command, payload=None):
        """Fungsi helper untuk mengirim perintah ke endpoint visi di backend."""
        try:
            requests.post(f"{self.base_url}/vision_command", json={"command": command, "payload": payload}, timeout=2)
        except requests.RequestException as e:
            print(f"Gagal mengirim perintah visi '{command}': {e}")

    def find_and_select_camera(self):
        """
        Secara otomatis meminta daftar kamera dari backend, lalu memilih yang pertama.
        """
        print("Mencari kamera aktif secara otomatis...")
        try:
            response = requests.get(f"{self.base_url}/list_cameras", timeout=3)
            if response.status_code == 200:
                self.camera_selector.blockSignals(True)
                self.camera_selector.clear()
                cameras = response.json().get("cameras", [])
                
                if cameras:
                    print(f"✅ Kamera ditemukan di indeks: {cameras}")
                    # Isi dropdown untuk pilihan manual nanti
                    self.camera_selector.addItems([f"Kamera {cam}" for cam in cameras])
                    
                    # --- LOGIKA OTOMATISASI ---
                    # Ambil indeks kamera pertama dari daftar dan kirim perintah ke backend
                    first_camera_index = cameras[0]
                    print(f"Otomatis memilih kamera pertama: Indeks {first_camera_index}")
                    self.send_vision_command("SELECT_CAMERA", first_camera_index)
                    # -------------------------
                else:
                    print("❌ Tidak ada kamera yang ditemukan oleh backend.")
                    self.camera_selector.addItem("Tidak ada kamera")
                
                self.camera_selector.blockSignals(False)
            else:
                print("Gagal mengambil daftar kamera dari backend.")
        except requests.RequestException as e:
            print(f"Gagal terhubung ke backend untuk mencari kamera: {e}")

    def refresh_camera_list(self):
        """Fungsi ini sekarang hanya untuk tombol Refresh manual."""
        self.find_and_select_camera() # Cukup panggil fungsi utama lagi

    def toggle_inversion(self):
        """Mengirim status tombol 'Invert' ke backend."""
        is_checked = self.invert_button.isChecked()
        self.send_vision_command("SET_INVERT", is_checked)
        if is_checked:
            self.invert_button.setStyleSheet("background-color: #e74c3c; color: white;")
        else:
            self.invert_button.setStyleSheet("")
            
    def select_camera_from_dropdown(self, index):
        """Mengirim indeks kamera yang DIPILIH PENGGUNA dari dropdown ke backend."""
        if index < 0: return
        cam_text = self.camera_selector.itemText(index)
        try:
            cam_index = int(cam_text.split(" ")[1])
            print(f"Pengguna memilih kamera dari dropdown: Indeks {cam_index}")
            self.send_vision_command("SELECT_CAMERA", cam_index)
        except (IndexError, ValueError):
            pass

    @Slot(np.ndarray)
    def set_frame(self, frame):
        if frame is None: return
        try:
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(qt_image)
            self.label.setPixmap(pixmap.scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except Exception:
            pass

    @Slot(str)
    def show_error_message(self, message):
        """Menampilkan pesan error jika koneksi video stream gagal."""
        self.label.setText(message)
        self.label.setStyleSheet("background-color: #c0392b; color: white; font-size: 14px; text-align: center;")

    def stop_camera(self):
        """Metode untuk menghentikan thread video saat aplikasi ditutup."""
        if self.stream_thread.isRunning():
            self.stream_thread.stop()
            self.stream_thread.wait(2000)

    @Slot(str)
    def set_mode(self, mode):
        """Mengirim status mode (AUTO/MANUAL) ke backend."""
        self.send_vision_command("SET_MODE", mode)