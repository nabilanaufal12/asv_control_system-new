import torch
import cv2
import serial
import time
import threading
from pathlib import Path
from pynput.keyboard import Key, Listener

# --- KONFIGURASI UTAMA (UBAH SESUAI KEBUTUHAN ANDA) ---

# Pengaturan Serial
SERIAL_PORT = '/dev/ttyUSB0'  # Ganti dengan port ESP32 Anda (misal: 'COM3' di Windows)
BAUD_RATE = 115200

# Pengaturan Kamera
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Pengaturan Model YOLOv5
YOLO_PATH = Path('yolov5') 
MODEL_PATH = YOLO_PATH / 'besto.pt'

# Pengaturan Kalibrasi Jarak
FOCAL_LENGTH_PIXELS = 600
REAL_WIDTH_CM = 20.0

# Kelas objek yang dianggap sebagai "bola"
BALL_CLASSES = ['red_buoy', 'green_buoy'] 

# --- VARIABEL GLOBAL UNTUK KONTROL ANTAR THREAD ---
is_a_pressed = False          # Status tombol 'a' untuk manual override
last_command_sent = None      # Mencegah spam perintah serial
running = True                # Flag untuk menghentikan thread dengan bersih

# --- FUNGSI HELPER ---

def clear_screen():
    """Fungsi untuk membersihkan layar konsol."""
    import os
    os.system('cls' if os.name == 'nt' else 'clear')

def estimate_distance(pixel_width, real_width_cm, focal_length_pixels):
    """Menghitung estimasi jarak ke objek."""
    if pixel_width == 0:
        return float('inf')
    return (real_width_cm * focal_length_pixels) / pixel_width

# --- FUNGSI UNTUK KEYBOARD LISTENER (PYNPUT) ---

def on_press(key):
    """Callback saat tombol ditekan."""
    global is_a_pressed
    try:
        if key.char == 'a' and not is_a_pressed:
            is_a_pressed = True
    except AttributeError:
        pass

def on_release(key):
    """Callback saat tombol dilepas."""
    global is_a_pressed, running
    try:
        if key.char == 'a':
            is_a_pressed = False
    except AttributeError:
        pass
    
    if key == Key.esc:
        print("\nTombol Esc ditekan. Meminta penutupan program...")
        running = False # Memberi sinyal ke semua loop untuk berhenti
        return False # Menghentikan listener

# --- FUNGSI UNTUK THREAD PEMBACA SERIAL ---

def serial_reader_thread(ser_instance):
    """Membaca dan menampilkan data telemetri dari ESP32."""
    global running
    while running:
        try:
            if ser_instance.in_waiting > 0:
                line = ser_instance.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("DATA:"):
                    # Logika parsing dan display dari skrip monitor Anda
                    parse_and_display_telemetry(line)
            time.sleep(0.05) # Jeda singkat agar tidak membebani CPU
        except Exception as e:
            if running: # Hanya tampilkan error jika kita tidak sedang dalam proses shutdown
                print(f"\nError di thread pembaca serial: {e}")
                break
    print("Thread pembaca serial dihentikan.")

def parse_and_display_telemetry(line):
    """Mem-parsing dan menampilkan telemetri ke konsol."""
    # Fungsi ini diambil langsung dari skrip monitor Anda
    data_string = line[5:]
    parts = data_string.split(',')
    mode = parts[0]
    
    clear_screen()
    print("--- NAVANTARA ASV MONITOR --- (Tekan 'q' di jendela kamera atau Esc di terminal untuk keluar)")
    
    if mode == "AUTO":
        if parts[1] == "AI_MODE_ACTIVATED":
            print("\n====================================")
            print("   MODE: AUTO [AI MODE ACTIVATED]")
            print("====================================")
            print("  ⚠️  KONTROL DIAMBIL ALIH OLEH LOGIKA PENGHINDARAN.")
        elif parts[1] == "RESUMED_FROM_AI":
            print("\n====================================")
            print("   MODE: AUTO [WAYPOINT RESUMED]")
            print("====================================")
            print("  ✅  Rintangan hilang. Melanjutkan navigasi waypoint.")
        elif len(parts) == 10:
            _, wp, dist, target_bearing, heading, error, servo, motor, speed, sats = parts
            print("\n====================================")
            print("   MODE: AUTO [WAYPOINT NAVIGATION]")
            print("====================================")
            print(f"  ● Waypoint: {wp} | Jarak: {dist} m")
            print(f"  ● Target: {target_bearing}° | Aktual: {heading}° (Error: {error}°)")
            print(f"  ● Servo: {servo}° | Motor: {motor}μs")
            print(f"  ● Kecepatan: {speed} km/h | Satelit: {sats}")
    elif mode == "MANUAL":
        if len(parts) == 8:
            _, lat, lon, heading, servo, motor, speed, sats = parts
            print("\n====================================")
            print("   MODE: MANUAL [REMOTE CONTROL]")
            print("====================================")
            print(f"  ● GPS: {lat}, {lon}")
            print(f"  ● Heading: {heading}° | Kecepatan: {speed} km/h")
            print(f"  ● Servo: {servo}° | Motor: {motor}μs")
            print(f"  ● Satelit: {sats}")
    else:
        print(f"\n[RAW TELEMETRY]: {line}")

# --- FUNGSI UTAMA ---

def main():
    """Fungsi utama untuk menjalankan semua komponen."""
    global last_command_sent, running

    # 1. Inisialisasi Serial
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ Berhasil terhubung ke port serial {SERIAL_PORT}")
        time.sleep(2)
    except serial.SerialException as e:
        print(f"🔥 GAGAL terhubung ke port serial {SERIAL_PORT}: {e}")
        return

    # 2. Muat Model YOLOv5
    try:
        if not YOLO_PATH.is_dir() or not MODEL_PATH.is_file():
            raise FileNotFoundError(f"Model '{MODEL_PATH}' atau direktori '{YOLO_PATH}' tidak ditemukan.")
        model = torch.hub.load(str(YOLO_PATH), 'custom', path=str(MODEL_PATH), source='local', trust_repo=True)
        print("✅ Model YOLOv5 'besto.pt' berhasil dimuat.")
    except Exception as e:
        print(f"🔥 GAGAL memuat model YOLOv5: {e}")
        if ser: ser.close()
        return

    # 3. Inisialisasi Kamera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"🔥 GAGAL membuka kamera dengan indeks {CAMERA_INDEX}.")
        if ser: ser.close()
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print(f"✅ Kamera dengan indeks {CAMERA_INDEX} berhasil dibuka.")

    # 4. Mulai Thread Pembaca Serial dan Keyboard Listener
    listener = Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    reader = threading.Thread(target=serial_reader_thread, args=(ser,))
    reader.start()
    
    print("\n🚀 Sistem Kontrol dan Monitor Aktif.")

    # 5. Loop Deteksi & Kontrol Utama
    try:
        while running:
            ret, frame = cap.read()
            if not ret:
                print("Gagal membaca frame, mengakhiri program.")
                running = False
                break

            results = model(frame)
            detections = results.pandas().xyxy[0]
            ball_detections = detections[detections['name'].isin(BALL_CLASSES)]
            closest_distance = float('inf')

            if not ball_detections.empty:
                best_detection = ball_detections.loc[ball_detections['confidence'].idxmax()]
                xmin, ymin, xmax, ymax = best_detection['xmin'], best_detection['ymin'], best_detection['xmax'], best_detection['ymax']
                pixel_width = xmax - xmin
                closest_distance = estimate_distance(pixel_width, REAL_WIDTH_CM, FOCAL_LENGTH_PIXELS)
                
                label = f"{best_detection['name']} ({closest_distance:.1f} cm)"
                cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 2)
                cv2.putText(frame, label, (int(xmin), int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Logika Prioritas Pengiriman Perintah
            if is_a_pressed:
                command_to_send = 'A'
                display_text = f"MANUAL OVERRIDE: '{command_to_send}'"
            else:
                command_to_send = 'A' if closest_distance < 80.0 else 'W'
                display_text = f"AUTO: Jarak {closest_distance:.1f}cm -> '{command_to_send}'"

            if command_to_send != last_command_sent:
                ser.write(f"{command_to_send}\n".encode('utf-8'))
                last_command_sent = command_to_send
            
            # Tampilkan status di jendela video
            cv2.putText(frame, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow('Kontrol & Monitor ASV', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                running = False
                break
    
    finally:
        print("\nMembersihkan dan menutup program...")
        running = False # Pastikan flag diset untuk menghentikan thread
        
        if listener.is_alive():
            listener.stop()
        
        reader.join() # Tunggu thread pembaca selesai

        if ser and ser.is_open:
            ser.write("W\n".encode('utf-8'))
            ser.close()
            print("Koneksi serial ditutup.")
        
        cap.release()
        cv2.destroyAllWindows()
        print("Jendela kamera ditutup.")
        print("Program selesai.")

if __name__ == '__main__':
    main()
