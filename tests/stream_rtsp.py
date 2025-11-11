import cv2
import numpy as np
import os
from flask import Flask, Response, render_template_string, url_for

app = Flask(__name__)

# --- 1. KONFIGURASI KAMERA & STREAMING (OPTIMAL) ---
# RESOLUSI RENDAH untuk kelancaran maksimum dan mengurangi beban CPU JPEG encoding
FRAME_WIDTH = 320 
FRAME_HEIGHT = 240
FLASK_PORT = 5000

# Pipeline GStreamer yang dioptimalkan untuk latensi rendah di Jetson
# 'drop=true max-buffers=1 sync=false' adalah kunci untuk mengurangi buffering/delay
#
# A. Untuk Kamera MIPI CSI (misalnya Raspberry Pi Camera Module V2):
CSI_PIPELINE = (
    f"nvarguscamerasrc sensor_id=0 ! "
    f"video/x-raw(memory:NVMM), width=1920, height=1080, format=NV12, framerate=30/1 ! "
    f"nvvidconv ! "
    f"video/x-raw(memory:NVMM), width={FRAME_WIDTH}, height={FRAME_HEIGHT}, format=NV12 ! " # Penskalaan di GPU
    f"nvvidconv ! "
    f"video/x-raw, format=BGR ! "
    f"appsink drop=true max-buffers=1 sync=false" 
)

# B. Untuk Webcam USB (V4L2):
USB_PIPELINE = (
    f"v4l2src device=/dev/video0 ! "
    f"video/x-raw, width={FRAME_WIDTH}, height={FRAME_HEIGHT}, framerate=30/1 ! "
    f"videoconvert ! "
    f"video/x-raw, format=BGR ! "
    f"appsink drop=true max-buffers=1 sync=false"
)

# PILIH PIPELINE: Ubah ke CSI_PIPELINE jika menggunakan kamera CSI
CAMERA_PIPELINE = USB_PIPELINE 
# -----------------------------------------------------------------

# Inisialisasi Objek VideoCapture
try:
    cap = cv2.VideoCapture(CAMERA_PIPELINE, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        print("? Kamera terbuka menggunakan GStreamer (Akselerasi Jetson)")
    else:
        raise IOError("Gagal membuka kamera melalui GStreamer.")
except Exception as e:
    print(f"? Error GStreamer: {e}. Mencoba fallback ke input V4L2 biasa.")
    cap = cv2.VideoCapture(0) # Fallback ke indeks kamera 0

if not cap.isOpened():
    print("FATAL ERROR: Tidak dapat membuka kamera sama sekali!")
    exit()

# Set parameter JPEG encoding untuk mengurangi beban CPU (kualitas 70%)
ENCODE_PARAM = [int(cv2.IMWRITE_JPEG_QUALITY), 70] 

def generate_frames():
    """Generator yang mengambil frame dari GStreamer/Kamera dan mengemasnya dalam format MJPEG."""
    while True:
        success, frame = cap.read()
        if not success:
            # Coba restart capture jika gagal membaca frame
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue
        
        # Encoding frame ke format JPEG (dilakukan di CPU, dioptimalkan dengan ENCODE_PARAM)
        ret, buffer = cv2.imencode('.jpg', frame, ENCODE_PARAM)
        frame_bytes = buffer.tobytes()

        # Yield frame dalam format MJPEG (Multipart Mixed Replace)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Rute utama yang menyajikan stream MJPEG."""
    # Memastikan header Cache-Control disetel untuk mencegah buffering browser
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame',
                    headers={'Cache-Control': 'no-cache, no-store, must-revalidate'})

@app.route('/')
def index():
    """Halaman HTML sederhana untuk menampilkan stream."""
    return render_template_string(
        f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Jetson Accelerated Stream</title>
        </head>
        <body>
            <h1>Stream Kamera Jetson Orin Nano (Optimized MJPEG 320x240)</h1>
            <p>Latensi rendah dicapai melalui GStreamer dan resolusi yang dioptimalkan.</p>
            <img src="{url_for('video_feed')}" width="{FRAME_WIDTH}" style="border: 2px solid black;"/>
            <p>Akses di: <b>http://[IP Jetson Anda]:{FLASK_PORT}</b></p>
        </body>
        </html>
        """
    )

if __name__ == '__main__':
    # Opsional: Tingkatkan prioritas proses untuk memastikan kelancaran
    try:
        os.nice(-10) 
        print("Prioritas proses ditingkatkan.")
    except Exception:
        print("Gagal mengatur prioritas (mungkin perlu hak akses root/sudo).")
        
    print(f"Memulai server di port {FLASK_PORT}...")
    print("Akses URL di atas di browser Anda.")
    # Jalankan server Flask agar dapat diakses dari jaringan lokal
    app.run(host='0.0.0.0', port=FLASK_PORT, debug=False)