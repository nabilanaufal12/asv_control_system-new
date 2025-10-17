import torch
import cv2
import serial
import time
import os
from pathlib import Path

# --- KONFIGURASI UTAMA (UBAH SESUAI KEBUTUHAN ANDA) ---

# Pengaturan Serial
SERIAL_PORT = '/dev/ttyUSB0'  # Ganti dengan port ESP32 Anda (misal: 'COM3' di Windows)
BAUD_RATE = 115200

# Pengaturan Kamera
CAMERA_INDEX = 0  # 0 untuk webcam internal, coba 1, 2, dst. jika ada kamera eksternal
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Pengaturan Model YOLOv5
# Skrip ini mengasumsikan folder 'yolov5' dan file 'besto.pt' berada di direktori yang sama
YOLO_PATH = Path('yolov5') 
MODEL_PATH = YOLO_PATH / 'besto.pt'

# Pengaturan Kalibrasi Jarak
# Anda HARUS mengkalibrasi nilai-nilai ini untuk mendapatkan akurasi yang baik
FOCAL_LENGTH_PIXELS = 600  # (Contoh: 600) - Kalibrasi dengan kamera Anda
REAL_WIDTH_CM = 20.0       # (Contoh: 20.0) - Lebar asli bola dalam cm

# Kelas objek yang dianggap sebagai "bola"
BALL_CLASSES = ['red_buoy', 'green_buoy'] 

# --- AKHIR KONFIGURASI ---


def estimate_distance(pixel_width, real_width_cm, focal_length_pixels):
    """Menghitung estimasi jarak ke objek."""
    if pixel_width == 0:
        return float('inf')
    return (real_width_cm * focal_length_pixels) / pixel_width

def main():
    """Fungsi utama untuk menjalankan deteksi dan komunikasi serial."""
    
    # 1. Inisialisasi Koneksi Serial
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"✅ Berhasil terhubung ke port serial {SERIAL_PORT}")
        time.sleep(2)  # Beri waktu pada ESP32 untuk siap
    except serial.SerialException as e:
        print(f"🔥 GAGAL terhubung ke port serial {SERIAL_PORT}: {e}")
        print("   Pastikan port sudah benar dan tidak digunakan oleh program lain.")
        return

    # 2. Muat Model YOLOv5
    try:
        # Periksa apakah path yolov5 ada
        if not YOLO_PATH.is_dir() or not MODEL_PATH.is_file():
             raise FileNotFoundError(f"Model '{MODEL_PATH}' atau direktori '{YOLO_PATH}' tidak ditemukan.")

        model = torch.hub.load(
            str(YOLO_PATH),
            'custom',
            path=str(MODEL_PATH),
            source='local',
            force_reload=True,
            trust_repo=True
        )
        print("✅ Model YOLOv5 'besto.pt' berhasil dimuat.")
    except Exception as e:
        print(f"🔥 GAGAL memuat model YOLOv5: {e}")
        if ser:
            ser.close()
        return

    # 3. Inisialisasi Kamera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"🔥 GAGAL membuka kamera dengan indeks {CAMERA_INDEX}.")
        if ser:
            ser.close()
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print(f"✅ Kamera dengan indeks {CAMERA_INDEX} berhasil dibuka.")

    last_command_sent = None # Untuk melacak perintah terakhir yang dikirim

    # 4. Loop Deteksi Utama
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Gagal membaca frame dari kamera. Mengakhiri program.")
                break

            # Lakukan inferensi
            results = model(frame)
            detections = results.pandas().xyxy[0]

            # Filter hanya untuk deteksi bola
            ball_detections = detections[detections['name'].isin(BALL_CLASSES)]

            closest_distance = float('inf')
            
            if not ball_detections.empty:
                # Temukan bola dengan confidence tertinggi
                best_detection = ball_detections.loc[ball_detections['confidence'].idxmax()]
                
                # Hitung jarak
                xmin, ymin, xmax, ymax = best_detection['xmin'], best_detection['ymin'], best_detection['xmax'], best_detection['ymax']
                pixel_width = xmax - xmin
                closest_distance = estimate_distance(pixel_width, REAL_WIDTH_CM, FOCAL_LENGTH_PIXELS)

                # Gambar bounding box dan info jarak
                label = f"{best_detection['name']} ({closest_distance:.1f} cm)"
                cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                cv2.putText(frame, label, (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Tentukan perintah yang akan dikirim ('A' atau 'W')
            command_to_send = 'A' if closest_distance < 80.0 else 'W'

            # Kirim perintah HANYA jika berbeda dari yang terakhir dikirim
            if command_to_send != last_command_sent:
                ser.write(f"{command_to_send}\n".encode('utf-8'))
                last_command_sent = command_to_send
                if command_to_send == 'A':
                    print(f"🧠 Jarak: {closest_distance:.1f} cm (< 80cm). Mengirim 'A' -> Mode AI")
                else:
                    print(f"🛰️  Jarak: {closest_distance:.1f} cm (>= 80cm). Mengirim 'W' -> Mode Waypoint")

            # Tampilkan frame
            cv2.imshow('Deteksi Bola - Tekan "q" untuk keluar', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Tombol 'q' ditekan. Menutup program.")
                break
    
    finally:
        # Bersihkan semua sumber daya saat keluar
        print("Membersihkan dan menutup...")
        if ser and ser.is_open:
            ser.write("W\n".encode('utf-8')) # Pastikan ESP kembali ke mode waypoint
            ser.close()
            print("Koneksi serial ditutup.")
        cap.release()
        cv2.destroyAllWindows()
        print("Jendela kamera ditutup.")

if __name__ == '__main__':
    main()