# gui/components/video_view.py
# --- MODIFIKASI FINAL: Logika hibrida untuk deteksi 1 ATAU 2 bola ---

import sys
import os
import cv2
import torch
import numpy as np
from pathlib import Path

if os.name == 'nt':
    import pathlib
    pathlib.PosixPath = pathlib.WindowsPath

try:
    CURRENT_FILE_DIR = Path(os.path.abspath(__file__)).resolve()
    PROJECT_ROOT = CURRENT_FILE_DIR.parents[2]
    YOLOV5_ROOT_PATH = PROJECT_ROOT / 'yolov5'
    if str(YOLOV5_ROOT_PATH) not in sys.path:
        sys.path.insert(0, str(YOLOV5_ROOT_PATH))

    from models.common import DetectMultiBackend
    from utils.general import non_max_suppression, scale_boxes, check_img_size
    from utils.torch_utils import select_device, smart_inference_mode
    from ultralytics.utils.plotting import Annotator, colors
    from utils.augmentations import letterbox
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"Peringatan: Gagal mengimpor modul YOLOv5: {e}")
    YOLO_AVAILABLE = False

from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QPushButton, QHBoxLayout, QComboBox
from PySide6.QtCore import QThread, Signal, Slot, Qt
from PySide6.QtGui import QImage, QPixmap

class DetectionThread(QThread):
    frame_ready = Signal(np.ndarray)
    vision_command_status = Signal(dict)

    def __init__(self, source_idx, weights_path, parent=None):
        super().__init__(parent)
        self.source_idx = source_idx
        self.weights_path = weights_path
        self.running = True
        self.mode_auto = False

    def get_score(self, ball, is_pair=False, other_ball=None):
        """Menghitung skor untuk satu bola atau sepasang."""
        size = (ball['xyxy'][2] - ball['xyxy'][0]) * (ball['xyxy'][3] - ball['xyxy'][1])
        if is_pair and other_ball:
            other_size = (other_ball['xyxy'][2] - other_ball['xyxy'][0]) * (other_ball['xyxy'][3] - other_ball['xyxy'][1])
            avg_y = (ball['center'][1] + other_ball['center'][1]) / 2
            return 0.6 * (size + other_size) + 0.4 * avg_y
        else:
            y_pos = ball['center'][1]
            return 0.6 * size + 0.4 * y_pos

    @smart_inference_mode()
    def run(self):
        cap = None
        try:
            print(f"Mencoba membuka kamera indeks: {self.source_idx} dengan backend MSMF...")
            cap = cv2.VideoCapture(self.source_idx, cv2.CAP_MSMF)
            if not cap.isOpened():
                print(f"ERROR: Tidak bisa membuka kamera indeks {self.source_idx} dengan MSMF.")
                return

            device = select_device('')
            model = DetectMultiBackend(weights=self.weights_path, device=device, fp16=False)
            stride, names, pt = model.stride, model.names, model.pt
            imgsz = check_img_size((640, 640), s=stride)
            model.warmup(imgsz=(1, 3, *imgsz))
            
            while self.running and cap.isOpened():
                ret, im0 = cap.read()
                if not ret: break

                img = letterbox(im0, imgsz, stride=stride, auto=pt)[0]
                img = img.transpose((2, 0, 1))[::-1]
                img = np.ascontiguousarray(img)
                im = torch.from_numpy(img).to(model.device).float() / 255.0
                if len(im.shape) == 3: im = im[None]

                pred = model(im, augment=False, visualize=False)
                pred = non_max_suppression(pred, conf_thres=0.4, iou_thres=0.45)

                for i, det in enumerate(pred):
                    annotator = Annotator(im0.copy(), line_width=2, example=str(names))
                    detected_red_balls, detected_green_balls = [], []
                    if len(det):
                        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                        for *xyxy, conf, cls in reversed(det):
                            class_name = names[int(cls)]
                            annotator.box_label(xyxy, f"{class_name} {conf:.2f}", color=colors(int(cls), True))
                            bbox_data = { 'xyxy': xyxy, 'center': (int((xyxy[0] + xyxy[2]) / 2), int((xyxy[1] + xyxy[3]) / 2)), 'class': class_name }
                            if "Red_Ball" in class_name: detected_red_balls.append(bbox_data)
                            elif "Green_Ball" in class_name: detected_green_balls.append(bbox_data)
                    
                    if self.mode_auto:
                        target_object = None
                        target_midpoint = None
                        lebar_asli_cm = 0

                        # --- PRIORITAS 1: CARI PASANGAN DUA BOLA ---
                        best_pair = None
                        if detected_red_balls and detected_green_balls:
                            best_score = -1
                            for red in detected_red_balls:
                                for green in detected_green_balls:
                                    score = self.get_score(red, is_pair=True, other_ball=green)
                                    if score > best_score:
                                        best_score = score
                                        best_pair = (red, green)
                        
                        if best_pair:
                            print("MODE: Dua Bola Terdeteksi.")
                            target_object = best_pair
                            red_ball, green_ball = best_pair
                            target_midpoint = ((red_ball['center'][0] + green_ball['center'][0]) // 2, (red_ball['center'][1] + green_ball['center'][1]) // 2)
                            lebar_asli_cm = 20.0 # Jarak antar bola
                        
                        # --- PRIORITAS 2: JIKA TIDAK ADA PASANGAN, CARI SATU BOLA TERBAIK ---
                        elif detected_red_balls or detected_green_balls:
                            print("MODE: Satu Bola Terdeteksi.")
                            all_balls = detected_red_balls + detected_green_balls
                            best_single_ball = max(all_balls, key=lambda b: self.get_score(b))
                            
                            target_object = (best_single_ball,) # Simpan sebagai tuple
                            target_midpoint = best_single_ball['center']
                            lebar_asli_cm = 10.0 # Lebar satu bola

                        # --- PROSES TARGET JIKA DITEMUKAN (BAIK 1 ATAU 2 BOLA) ---
                        if target_object:
                            # Tentukan bounding box gabungan dari target
                            if len(target_object) == 2: # Jika pasangan
                                x1 = min(target_object[0]['xyxy'][0], target_object[1]['xyxy'][0])
                                y1 = min(target_object[0]['xyxy'][1], target_object[1]['xyxy'][1])
                                x2 = max(target_object[0]['xyxy'][2], target_object[1]['xyxy'][2])
                                y2 = max(target_object[0]['xyxy'][3], target_object[1]['xyxy'][3])
                            else: # Jika satu bola
                                x1, y1, x2, y2 = target_object[0]['xyxy']

                            PANJANG_FOKUS_PIKSEL = 600
                            lebar_objek_piksel = x2 - x1
                            jarak_estimasi_cm = (lebar_asli_cm * PANJANG_FOKUS_PIKSEL) / lebar_objek_piksel if lebar_objek_piksel > 0 else 0
                            
                            annotator.box_label([x1, y1, x2, y2], f"Jarak: {jarak_estimasi_cm:.1f} cm")

                            batas_posisi_y = im0.shape[0] * 0.5
                            TARGET_JARAK_CM = 100.0 

                            if jarak_estimasi_cm > 0 and jarak_estimasi_cm < TARGET_JARAK_CM and y1 > batas_posisi_y:
                                print(f"ZONA AKTIVASI TERPENUHI (Jarak: {jarak_estimasi_cm:.1f} cm). Mengikuti objek.")
                                degree = self.calculate_degree(im0, target_midpoint)
                                servo_angle, motor_pwm = self.convert_degree_to_actuators(degree)
                                print(f"   => Perintah Dihitung: Motor PWM={motor_pwm}, Servo Angle={servo_angle}")
                                self.vision_command_status.emit({'status': 'ACTIVE', 'command': f"S{motor_pwm};D{servo_angle}\n"})
                            else:
                                self.vision_command_status.emit({'status': 'INACTIVE'})
                        else:
                            self.vision_command_status.emit({'status': 'INACTIVE'})
                    
                    self.frame_ready.emit(annotator.result())
        except Exception as e:
            print(f"Error di dalam thread deteksi: {e}")
        finally:
            if cap and cap.isOpened():
                cap.release()
                print("Sumber daya kamera berhasil dilepaskan.")
            print("Pembersihan thread deteksi selesai.")

    def stop(self): self.running = False

    def calculate_degree(self, frame, midpoint):
        h, w, _ = frame.shape
        bar_y, bar_left, bar_right = h - 50, 50, w - 50
        bar_width = bar_right - bar_left
        relative_pos = np.clip((midpoint[0] - bar_left) / bar_width, 0, 1)
        return int(relative_pos * 180)

    def convert_degree_to_actuators(self, degree):
        error = degree - 90
        servo_angle = 90 - (error / 90.0) * 45
        servo_angle = max(0, min(180, int(servo_angle)))
        speed_reduction = abs(error) / 90.0
        motor_pwm = 1600 - (speed_reduction * 100)
        return int(servo_angle), int(motor_pwm)

class VideoView(QWidget):
    vision_status_updated = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.detection_thread = None
        self.label = QLabel("Kamera nonaktif.")
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setMinimumSize(640, 480)
        self.label.setStyleSheet("background-color: #2c3e50; color: white;")
        self.camera_selector = QComboBox()
        self.refresh_button = QPushButton("Refresh List")
        self.start_stop_button = QPushButton("Start Camera")
        control_layout = QHBoxLayout()
        control_layout.addWidget(QLabel("Sumber Kamera:"))
        control_layout.addWidget(self.camera_selector, 1)
        control_layout.addWidget(self.refresh_button)
        control_layout.addWidget(self.start_stop_button)
        main_layout = QVBoxLayout(self)
        main_layout.addLayout(control_layout)
        main_layout.addWidget(self.label, 1)
        self.refresh_button.clicked.connect(self.list_cameras)
        self.start_stop_button.clicked.connect(self.toggle_camera)
        if not YOLO_AVAILABLE:
            self.label.setText("Modul YOLOv5 tidak ditemukan.")
            self.start_stop_button.setEnabled(False)
        else:
            self.list_cameras()
            
    def list_cameras(self):
        self.camera_selector.clear()
        index = 0; arr = []
        while True:
            cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
            if not cap.isOpened(): break
            arr.append(index)
            cap.release()
            index += 1
        if arr:
            self.camera_selector.addItems([f"Kamera {i}" for i in arr])
            self.start_stop_button.setEnabled(True)
        else:
            self.camera_selector.addItem("Tidak ada kamera")
            self.start_stop_button.setEnabled(False)

    def toggle_camera(self):
        if self.detection_thread and self.detection_thread.isRunning():
            self.stop_camera()
        else:
            self.start_camera()

    def start_camera(self):
        if self.detection_thread and self.detection_thread.isRunning(): return
        if "Kamera" not in self.camera_selector.currentText(): return
        source_str = self.camera_selector.currentText().split(" ")[1]
        source_idx = int(source_str)
        weights_path = str(YOLOV5_ROOT_PATH / "besto.pt")
        
        print(f"Memulai thread kamera baru untuk device {source_idx}...")
        self.detection_thread = DetectionThread(source_idx=source_idx, weights_path=weights_path, parent=self)
        self.detection_thread.frame_ready.connect(self.set_frame)
        self.detection_thread.vision_command_status.connect(self.vision_status_updated.emit)
        self.detection_thread.start()
        self.start_stop_button.setText("Stop Camera")

    def stop_camera(self):
        if self.detection_thread and self.detection_thread.isRunning():
            print("Memulai prosedur penghentian kamera...")
            self.detection_thread.stop()
            self.detection_thread.wait()
            self.detection_thread = None
            print("Thread kamera berhasil dihentikan.")
            
        self.start_stop_button.setText("Start Camera")
        self.label.setText("Kamera nonaktif.")

    @Slot(np.ndarray)
    def set_frame(self, frame):
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image).scaled(self.label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.stop_camera()
        super().closeEvent(event)

    @Slot(str)
    def set_mode(self, mode):
        if self.detection_thread and self.detection_thread.isRunning():
            self.detection_thread.mode_auto = (mode == "AUTO")