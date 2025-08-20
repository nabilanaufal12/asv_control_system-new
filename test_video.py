import cv2
import torch

# Load model YOLOv5 (pretrained COCO)
print("[Vision] Loading YOLOv5 model (CPU)...")
model = torch.hub.load("ultralytics/yolov5", "yolov5s", pretrained=True)
model.to("cpu")  # pakai CPU saja

# Buka kamera (0 = default webcam)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Kamera tidak bisa dibuka")
    exit()

print("[Vision] Tekan 'q' untuk keluar")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Gagal membaca frame")
        break

    # Deteksi objek
    results = model(frame)

    # Render hasil deteksi ke frame
    annotated_frame = results.render()[0]

    # Tampilkan ke window
    cv2.imshow("YOLOv5 Detection", annotated_frame)

    # Tekan q untuk keluar
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
