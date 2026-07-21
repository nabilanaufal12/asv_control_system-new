import os

# Fix untuk SSH tanpa display — harus di-set SEBELUM import cv2
os.environ["QT_QPA_PLATFORM"] = "offscreen"

import cv2  # noqa: E402
from ultralytics import YOLO  # noqa: E402

# Tambahkan ini di vision_service.py Anda nanti
CLASS_NAMES = {
    0: "bola-biru",
    1: "kotak-biru",
    2: "bola-hijau",
    3: "kotak-hijau",
    4: "bola-merah",
}

# Gunakan engine yang dibuild di Jetson INI (bukan dari device lain)
MODEL_PATH = "/home/navantara/navantara/src/navantara_backend/vision/best.engine"


def main():
    print(f"Memuat model TensorRT: {MODEL_PATH}")

    if not os.path.exists(MODEL_PATH):
        print(f"Error: Model tidak ditemukan di {MODEL_PATH}")
        print(
            "Pastikan sudah menjalankan trtexec untuk convert ONNX → engine di Jetson ini."
        )
        return

    try:
        model = YOLO(MODEL_PATH, task="detect")
    except Exception as e:
        print(f"Error memuat model: {e}")
        return

    print("Membuka kamera (webcam 0)...")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Tidak dapat membuka kamera.")
        print(
            "Silakan coba mengubah index VideoCapture(0) jika Anda memiliki kamera eksternal."
        )
        return

    print("Inferensi berjalan (headless mode, tanpa GUI).")
    print("Tekan Ctrl+C untuk berhenti.\n")

    frame_count = 0
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Gagal mengambil frame dari kamera.")
                break

            # Jalankan inferensi
            results = model.predict(source=frame, conf=0.5, verbose=False)

            # Tampilkan hasil deteksi di terminal
            for result in results:
                boxes = result.boxes
                if len(boxes) > 0:
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        label_name = model.names.get(cls_id, f"class_{cls_id}")
                        print(
                            f"[Frame {frame_count}] Terdeteksi: {label_name} ({conf:.2f}) @ [{x1},{y1},{x2},{y2}]"
                        )

            frame_count += 1

    except KeyboardInterrupt:
        print(f"\nDihentikan. Total frame diproses: {frame_count}")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
