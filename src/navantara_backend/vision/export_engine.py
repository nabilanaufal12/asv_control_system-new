from ultralytics import YOLO

# 1. Load model .pt
model = YOLO("src/navantara_backend/vision/best.pt")

# 2. Export ke TensorRT dengan imgsz 320
# Proses ini memakan waktu 5-10 menit di Jetson
model.export(format="engine", imgsz=320, device=0)
