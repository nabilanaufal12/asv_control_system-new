from flask import Flask, Response
import cv2
import time
import sys
import numpy as np

app = Flask(__name__)

# --- KONFIGURASI KAMERA & STREAMING ---
CAMERA_INDEX_1 = 0 
CAMERA_INDEX_2 = 2

# Resolusi target rendah untuk kelancaran
TARGET_WIDTH = 320
TARGET_HEIGHT = 240

# Kualitas kompresi JPEG rendah (50%)
JPEG_QUALITY = 50 

# Parameter encoding JPEG
ENCODE_PARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY] 

# --- KONFIGURASI DETEKSI JARI (Perlu disesuaikan jika deteksi tidak akurat) ---
# Rentang warna kulit di HSV: [H_min, S_min, V_min] hingga [H_max, S_max, V_max]
# Ini adalah rentang umum. SESUAIKAN JIKA PERLU!
LOWER_SKIN = np.array([0, 20, 70], dtype=np.uint8)
UPPER_SKIN = np.array([20, 255, 255], dtype=np.uint8)
MIN_CONTOUR_AREA = 1000  # Luas minimum kontur tangan
MIN_DEFECT_DEPTH = 20 * 256 # Kedalaman minimum lekukan (defect) untuk dianggap sebagai pemisah jari

# --- FUNGSI INISIALISASI ---

def init_camera(index):
    """Menginisialisasi objek VideoCapture dengan V4L2 dan pengaturan resolusi."""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

    # Atur resolusi
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, TARGET_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, TARGET_HEIGHT)

    if not cap.isOpened():
        print(f"FATAL ERROR: Gagal membuka kamera USB pada indeks {index}.")
        return None
    
    # Pemanasan
    for _ in range(5):
        cap.read() 
        
    print(f"SUCCESS: Kamera USB pada indeks {index} berhasil dibuka.")
    return cap

# Inisialisasi kedua kamera
camera_1 = init_camera(CAMERA_INDEX_1)
camera_2 = init_camera(CAMERA_INDEX_2)

if camera_1 is None or camera_2 is None:
    print("Aplikasi dimatikan karena salah satu kamera gagal diinisialisasi. Cek indeks dan izin.")
    sys.exit(1)

# --- FUNGSI DETEKSI JARI ---

def detect_fingers(frame):
    """Mendeteksi jari menggunakan warna kulit, kontur, dan convexity defects."""
    
    # Konversi ke HSV dan terapkan mask warna kulit
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_SKIN, UPPER_SKIN)

    # Operasi Morfologi untuk membersihkan noise
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.erode(mask,kernel,iterations = 1)
    mask = cv2.dilate(mask,kernel,iterations = 1)
    
    # Temukan Kontur
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    fingertips_count = 0
    
    if contours:
        # Cari kontur terbesar (asumsi tangan)
        max_contour = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(max_contour) > MIN_CONTOUR_AREA:
            
            # Convex Hull
            hull = cv2.convexHull(max_contour, returnPoints=False)
            
            if len(hull) > 2:
                # Convexity Defects
                defects = cv2.convexityDefects(max_contour, hull)

                if defects is not None:
                    # Hitung defects yang dalam (mewakili celah antar jari)
                    valid_defects = 0
                    for i in range(defects.shape[0]):
                        s, e, f, d = defects[i, 0]
                        start = tuple(max_contour[s][0])
                        far = tuple(max_contour[f][0])
                        
                        # Hanya pertimbangkan defects yang memiliki kedalaman cukup
                        if d > MIN_DEFECT_DEPTH:
                            # Gambar lingkaran di ujung jari (titik start)
                            cv2.circle(frame, start, 5, (0, 255, 255), -1)
                            valid_defects += 1
                            
                    # Jumlah jari = jumlah defects + 1 (atau hanya hitung defects yang valid)
                    # Pendekatan sederhana: 1 defect = 1 jari, 2 defects = 2 jari.
                    # Asumsi: Setiap defect valid diapit oleh dua jari (sehingga jumlah jari adalah defects + 1)
                    # Karena ini sederhana, kita ambil jumlah defects valid sebagai perkiraan jari
                    # Kita juga bisa menggunakan heuristic: jumlah jari = valid_defects + 1 (sampai 5)
                    
                    fingertips_count = valid_defects + 1 # Jari = celah + 1
                    
                    # Batasi maksimal 5 jari
                    if fingertips_count > 5:
                        fingertips_count = 5 
                    elif valid_defects == 0 and cv2.contourArea(max_contour) > 15000:
                        # Jika tidak ada defect tapi kontur besar, asumsikan telapak tangan (1 jari/telapak)
                        fingertips_count = 1
                    elif valid_defects == 0:
                         fingertips_count = 0


    # Tambahkan teks jumlah jari yang terdeteksi
    cv2.putText(frame, f'Fingers: {fingertips_count}', (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    
    return frame


# --- GENERATOR DAN ROUTE FLASK ---

def generate_frames(camera):
    """Generator universal untuk kedua kamera."""
    while True:
        success, frame = camera.read()
        if not success:
            time.sleep(1)
            continue
        
        # PROSES DETEKSI JARI
        processed_frame = detect_fingers(frame.copy())
        
        # Encoding Frame
        ret, buffer = cv2.imencode('.jpg', processed_frame, ENCODE_PARAM)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed/1')
def video_feed_1():
    """Route untuk stream kamera 1."""
    return Response(generate_frames(camera_1),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed/2')
def video_feed_2():
    """Route untuk stream kamera 2."""
    return Response(generate_frames(camera_2),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    """Halaman HTML untuk menampilkan kedua stream."""
    return f"""
    <html>
        <head>
            <title>Dual Jetson Orin Nano Stream - Finger Detection</title>
        </head>
        <body>
            <h1>Kamera 1 (Indeks {CAMERA_INDEX_1})</h1>
            <img src="/video_feed/1" style="max-width: 48%; border: 2px solid blue; margin-right: 1%;">
            
            <h1>Kamera 2 (Indeks {CAMERA_INDEX_2})</h1>
            <img src="/video_feed/2" style="max-width: 48%; border: 2px solid red;">
            
            <p>Konfigurasi: Resolusi {TARGET_WIDTH}x{TARGET_HEIGHT}, Kualitas JPEG {JPEG_QUALITY}%</p>
            <p><strong>Catatan:</strong> Deteksi jari berbasis warna kulit dan kontur. Akurasi sangat bergantung pada pencahayaan dan latar belakang. Pastikan tangan Anda memiliki kontras yang baik.</p>
        </body>
    </html>
    """

if __name__ == '__main__':
    try:
        print("Flask server berjalan di http://<IP_ADDRESS_JETSON>:5000/")
        # Menggunakan 'threaded=True' untuk menangani kedua stream secara paralel
        app.run(host='0.0.0.0', port=5000, threaded=True, debug=False)
    finally:
        # Pembersihan
        if camera_1 is not None and camera_1.isOpened():
            camera_1.release()
        if camera_2 is not None and camera_2.isOpened():
            camera_2.release()
        print("\nMelepaskan sumber kamera dan mematikan aplikasi.")