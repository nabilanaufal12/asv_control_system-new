import cv2

print("Mencari kamera yang tersedia...")
index = 0
available_cameras = []

# Cek 10 indeks pertama
while index < 10:
    # Coba buka kamera dengan backend DSHOW
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    if cap.isOpened():
        print(f"✅ Kamera ditemukan di indeks {index}")
        available_cameras.append(index)
        cap.release()
    else:
        print(f"❌ Tidak ada kamera di indeks {index}")
    index += 1

print("\n--- Hasil ---")
if available_cameras:
    print(f"Kamera yang valid ditemukan di indeks: {available_cameras}")
    print(f"Silakan gunakan salah satu dari angka ini di 'vision_service.py'.")
else:
    print("Tidak ada kamera yang terdeteksi. Pastikan kamera terhubung dan driver terinstal.")