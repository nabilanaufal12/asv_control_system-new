import cv2
import numpy as np
from flask import Flask, Response

# --- Konfigurasi ---
# Ganti 0 dengan indeks kamera Anda (misal 0 untuk kamera USB)
# Untuk kamera CSI, gunakan string GStreamer:
# cap = cv2.VideoCapture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)
CAMERA_INDEX = 2
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FLASK_PORT = 5000

# Inisialisasi Aplikasi Flask
app = Flask(__name__)

# Inisialisasi Objek VideoCapture
# Gunakan GStreamer untuk memaksimalkan akselerasi pada Jetson
try:
    # Coba menggunakan pipeline GStreamer untuk input CSI/USB yang diakselerasi
    gstreamer_pipeline = (
        f"v4l2src device=/dev/video{CAMERA_INDEX} ! "
        f"video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=YUYV ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=1"
    )
    cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
    print("Menggunakan GStreamer Pipeline...")
except:
    # Fallback ke input OpenCV standar jika GStreamer gagal
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    print("Menggunakan Standar OpenCV...")


if not cap.isOpened():
    raise IOError("Tidak dapat membuka kamera!")

def generate_frames():
    """Generator yang mengambil frame dari kamera dan mengemasnya dalam format JPEG."""
    while True:
        # Ambil frame
        success, frame = cap.read()
        if not success:
            break
        
        # Encoding frame ke format JPEG
        # Encoding JPEG ini yang TIDAK menggunakan akselerasi GPU (dilakukan di CPU)
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # Yield frame dalam format MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Rute streaming MJPEG."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    """Halaman HTML sederhana untuk menampilkan stream."""
    return (
        f"<h1>Stream Kamera Jetson Orin Nano</h1>"
        f"<img src='{app.url_for('video_feed')}' width='{FRAME_WIDTH}'/>"
    )

if __name__ == '__main__':
    # Pastikan server dapat diakses dari jaringan lokal
    app.run(host='0.0.0.0', port=FLASK_PORT, debug=False)