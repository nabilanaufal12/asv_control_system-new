# gui/components/camera/detection_thread.py
# --- FINAL LENGKAP: Dengan logika navigasi AUTO yang diisi kembali ---

import sys
import os
import cv2
import torch
import numpy as np
import time
from pathlib import Path

# --- JARING PENGAMAN: Tambahkan patch ini di baris paling atas ---
if os.name == 'nt': # Hanya untuk Windows
    import pathlib
    pathlib.PosixPath = pathlib.WindowsPath

# Impor fungsi overlay dari file terpisah di folder yang sama
from .overlay_utils import draw_geotag_overlay

# Blok impor untuk YOLOv5
try:
    CURRENT_FILE_DIR = Path(os.path.abspath(__file__)).resolve()
    PROJECT_ROOT = CURRENT_FILE_DIR.parents[3]
    YOLO_AVAILABLE = True
except ImportError as e:
    print(f"Peringatan: Gagal mengimpor modul: {e}")
    YOLO_AVAILABLE = False

from PySide6.QtCore import QThread, Signal, Slot

class DetectionThread(QThread):
    frame_ready = Signal(np.ndarray)
    vision_command_status = Signal(dict)

    def __init__(self, source_idx, weights_path, parent=None):
        super().__init__(parent)
        self.source_idx, self.weights_path = source_idx, weights_path
        self.running, self.mode_auto, self.is_inverted = True, False, False
        
        self.snapshot_dir = PROJECT_ROOT / "snapshots"
        os.makedirs(self.snapshot_dir, exist_ok=True)
        print(f"Gambar akan disimpan di: {self.snapshot_dir}")
        
        self.latest_telemetry = {}

    @Slot(dict)
    def update_telemetry(self, data):
        self.latest_telemetry = data

    def get_score(self, ball, is_pair=False, other_ball=None):
        size = (ball['xyxy'][2] - ball['xyxy'][0]) * (ball['xyxy'][3] - ball['xyxy'][1])
        if is_pair and other_ball:
            other_size = (other_ball['xyxy'][2] - other_ball['xyxy'][0]) * (other_ball['xyxy'][3] - other_ball['xyxy'][1])
            avg_y = (ball['center'][1] + other_ball['center'][1]) / 2
            return 0.6 * (size + other_size) + 0.4 * avg_y
        else:
            y_pos = ball['center'][1]
            return 0.6 * size + 0.4 * y_pos
            
    def run(self):
        cap = None
        try:
            print("Mempersiapkan model YOLOv5...")
            # Menggunakan torch.hub.load dengan force_reload=True untuk membersihkan cache
            model = torch.hub.load(
                'ultralytics/yolov5',
                'custom',
                path=self.weights_path,
                force_reload=True # Ini akan memperbaiki error PosixPath
            )
            model.conf = 0.4
            model.iou = 0.45
            
            print(f"Mencoba membuka kamera indeks: {self.source_idx}...")
            cap = cv2.VideoCapture(self.source_idx, cv2.CAP_MSMF)
            if not cap.isOpened():
                print(f"ERROR: Tidak bisa membuka kamera {self.source_idx}.")
                return
            
            print("Kamera dan model berhasil dimuat. Memulai deteksi...")
            while self.running and cap.isOpened():
                ret, im0 = cap.read()
                if not ret: break

                results = model(im0)
                
                results.render()
                annotated_frame = results.ims[0]
                
                detected_red_buoys, detected_green_buoys = [], []
                detected_green_boxes, detected_blue_boxes = [], []

                df = results.pandas().xyxy[0]
                for _, row in df.iterrows():
                    class_name = row['name']
                    bbox_data = {
                        'xyxy': [row['xmin'], row['ymin'], row['xmax'], row['ymax']],
                        'center': (int((row['xmin'] + row['xmax']) / 2), int((row['ymin'] + row['ymax']) / 2)),
                        'class': class_name
                    }
                    
                    if "red_buoy" in class_name: detected_red_buoys.append(bbox_data)
                    elif "green_buoy" in class_name: detected_green_buoys.append(bbox_data)
                    if "green_box" in class_name: detected_green_boxes.append(bbox_data)
                    elif "blue_box" in class_name: detected_blue_boxes.append(bbox_data)

                mission_name = None
                if detected_green_boxes: mission_name = "Surface Imaging"
                elif detected_blue_boxes: mission_name = "Underwater Imaging"
                
                if mission_name:
                    snapshot_image = draw_geotag_overlay(self.latest_telemetry, im0, mission_type=mission_name)
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"snapshot_{mission_name.replace(' ','_')}_{timestamp}.jpg"
                    filepath = self.snapshot_dir / filename
                    cv2.imwrite(str(filepath), snapshot_image)
                    print(f"✅ SNAPSHOT DIAMBIL: '{mission_name}' terdeteksi. Disimpan sebagai {filename}")

                # --- LOGIKA NAVIGASI (HANYA AKTIF SAAT MODE AUTO) ---
                if self.mode_auto:
                    target_object, target_midpoint, lebar_asli_cm = None, None, 0
                    best_pair = None
                    if detected_red_buoys and detected_green_buoys:
                        best_score = -1
                        for red in detected_red_buoys:
                            for green in detected_green_buoys:
                                score = self.get_score(red, is_pair=True, other_ball=green)
                                if score > best_score:
                                    best_score = score
                                    best_pair = (red, green)
                    if best_pair:
                        print("MODE: Dua Bola Terdeteksi.")
                        target_object, red_ball, green_ball = best_pair, best_pair[0], best_pair[1]
                        target_midpoint = ((red_ball['center'][0] + green_ball['center'][0]) // 2, (red_ball['center'][1] + green_ball['center'][1]) // 2)
                        lebar_asli_cm = 20.0
                    elif detected_red_buoys or detected_green_buoys:
                        print("MODE: Satu Bola Terdeteksi.")
                        all_balls = detected_red_buoys + detected_green_buoys
                        best_single_ball = max(all_balls, key=lambda b: self.get_score(b))
                        target_object, target_midpoint, lebar_asli_cm = (best_single_ball,), best_single_ball['center'], 10.0

                    if target_object:
                        if len(target_object) == 2:
                            x1, y1 = min(target_object[0]['xyxy'][0], target_object[1]['xyxy'][0]), min(target_object[0]['xyxy'][1], target_object[1]['xyxy'][1])
                            x2, y2 = max(target_object[0]['xyxy'][2], target_object[1]['xyxy'][2]), max(target_object[0]['xyxy'][3], target_object[1]['xyxy'][3])
                        else:
                            x1, y1, x2, y2 = target_object[0]['xyxy']
                        
                        # Buat annotator baru khusus untuk menggambar kotak jarak
                        # agar tidak tercampur dengan hasil render()
                        distance_annotator = Annotator(annotated_frame, line_width=2, example="Jarak")
                        
                        PANJANG_FOKUS_PIKSEL, lebar_objek_piksel = 600, x2 - x1
                        jarak_estimasi_cm = (lebar_asli_cm * PANJANG_FOKUS_PIKSEL) / lebar_objek_piksel if lebar_objek_piksel > 0 else 0
                        distance_annotator.box_label([x1, y1, x2, y2], f"Jarak: {jarak_estimasi_cm:.1f} cm")
                        annotated_frame = distance_annotator.result()


                        batas_posisi_y, TARGET_JARAK_CM = im0.shape[0] * 0.5, 100.0
                        if jarak_estimasi_cm > 0 and jarak_estimasi_cm < TARGET_JARAK_CM and y1 > batas_posisi_y:
                            print(f"ZONA AKTIVASI TERPENUHI (Jarak: {jarak_estimasi_cm:.1f} cm). Mengikuti objek.")
                            degree = self.calculate_degree(im0, target_midpoint)
                            servo_angle, motor_pwm = self.convert_degree_to_actuators(degree)
                            print(f"   => Perintah Dihitung: Motor PWM={motor_pwm}, Servo Angle={servo_angle}")
                            self.vision_command_status.emit({'status': 'ACTIVE', 'command': f"S{motor_pwm};D{servo_angle}\n"})
                        else: self.vision_command_status.emit({'status': 'INACTIVE'})
                    else: self.vision_command_status.emit({'status': 'INACTIVE'})

                self.frame_ready.emit(annotated_frame)
                
        except Exception as e:
            print(f"Error di dalam thread deteksi: {e}")
        finally:
            if cap and cap.isOpened(): cap.release()
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
        if self.is_inverted: error = -error
        servo_angle = 90 - (error / 90.0) * 45
        servo_angle = max(0, min(180, int(servo_angle)))
        speed_reduction = abs(error) / 90.0
        motor_pwm = 1600 - (speed_reduction * 100)
        return int(servo_angle), int(motor_pwm)