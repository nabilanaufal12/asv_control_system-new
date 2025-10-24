from flask import Flask, Response
import cv2
import threading

# Variabel global untuk menyimpan frame terakhir
output_frame = None
lock = threading.Lock()

app = Flask(__name__)

# Inisialisasi webcam dan atur propertinya
kamera = cv2.VideoCapture(0)
kamera.set(cv2.CAP_PROP_FPS, 15)  # Mengatur FPS


def update_frame():
    """Thread untuk mengambil dan mengubah ukuran frame."""
    global output_frame, lock
    while True:
        sukses, frame = kamera.read()
        if not sukses:
            break

        # Ubah ukuran frame secara paksa (lebih andal)
        frame_kecil = cv2.resize(frame, (480, 320))

        with lock:
            output_frame = frame_kecil.copy()


def hasilkan_frame():
    """Thread untuk melayani frame ke browser."""
    global output_frame, lock
    while True:
        with lock:
            if output_frame is None:
                continue

            # Gunakan kompresi WebP yang lebih efisien
            ret, buffer = cv2.imencode(
                ".webp", output_frame, [cv2.IMWRITE_WEBP_QUALITY, 50]
            )
            if not ret:
                continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/webp\r\n\r\n" + buffer.tobytes() + b"\r\n"
        )


@app.route("/video")
def video_feed():
    return Response(
        hasilkan_frame(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    # Buat dan jalankan thread kamera
    thread_kamera = threading.Thread(target=update_frame)
    thread_kamera.daemon = True
    thread_kamera.start()

    # Jalankan server Flask
    app.run(host="0.0.0.0", port=5000)
