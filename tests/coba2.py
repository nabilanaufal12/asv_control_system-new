import cv2
import numpy as np

DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 360

# Buka Kamera 0 (USB) di /dev/video0
print("Membuka Kamera 0 di /dev/video0")
cap0 = cv2.VideoCapture(0)
cap0.set(cv2.CAP_PROP_FRAME_WIDTH, DISPLAY_WIDTH)
cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, DISPLAY_HEIGHT)

# Buka Kamera 1 (USB) di /dev/video2 
# (Saya tebak '2' karena Anda mencoba sensor-id=2)
# Jika gagal, coba ganti ke cv2.VideoCapture(1)
print("Membuka Kamera 1 di /dev/video2")
cap1 = cv2.VideoCapture(2) 
cap1.set(cv2.CAP_PROP_FRAME_WIDTH, DISPLAY_WIDTH)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, DISPLAY_HEIGHT)

# Cek apakah kamera terbuka
if not cap0.isOpened():
    print("Error: Tidak bisa membuka Kamera 0 (/dev/video0).")
    exit()
if not cap1.isOpened():
    print("Error: Tidak bisa membuka Kamera 1 (/dev/video2).")
    exit()

print("Kedua kamera USB berhasil dibuka. Tekan 'q' untuk keluar.")

window_name = "Tampilan 2 Kamera USB"
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

while True:
    ret0, frame0 = cap0.read()
    ret1, frame1 = cap1.read()

    if not ret0 or not ret1:
        print("Error: Gagal membaca frame.")
        break

    # Gabungkan
    combined_frame = np.hstack((frame0, frame1))
    cv2.imshow(window_name, combined_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Menutup kamera...")
cap0.release()
cap1.release()
cv2.destroyAllWindows()