# src/scripts/check_labels.py
from ultralytics import YOLO

# Pastikan path model benar
MODEL_PATH = "src/navantara_backend/vision/best.pt"

try:
    print(f"Memuat model dari {MODEL_PATH}...")
    model = YOLO(MODEL_PATH)
    print("\n=== DAFTAR KELAS DI MODEL ===")
    print(model.names)
    print("=============================\n")
    print("Gunakan nama-nama di atas untuk mengisi LABEL_MAP di vision_service.py")
except Exception as e:
    print(f"Error: {e}")
