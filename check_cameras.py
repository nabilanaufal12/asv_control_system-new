# check_cameras.py
import cv2

def find_available_cameras(max_cameras_to_check=10):
    """
    Mendeteksi dan menampilkan semua indeks kamera yang tersedia.
    """
    available_cameras = []
    print("Mencari kamera yang tersedia...")
    for i in range(max_cameras_to_check):
        # Menggunakan CAP_DSHOW untuk konsistensi dengan aplikasi utama di Windows
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap.isOpened():
            print(f"✅ Kamera ditemukan di indeks: {i}")
            available_cameras.append(i)
            # Penting: Lepaskan kamera agar bisa digunakan oleh program lain
            cap.release()
        else:
            print(f"-- Tidak ada kamera di indeks: {i}")
            
    if not available_cameras:
        print("\n❌ KRITIS: Tidak ada kamera yang terdeteksi oleh OpenCV.")
        print("Pastikan kamera terhubung dengan baik dan driver sudah terinstal.")
    else:
        print(f"\nRingkasan: Ditemukan kamera di indeks berikut --> {available_cameras}")
        print("Gunakan salah satu dari indeks ini di file config.json Anda.")

if __name__ == "__main__":
    find_available_cameras()
