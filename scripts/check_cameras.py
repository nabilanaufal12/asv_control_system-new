# scripts/check_cameras.py
# --- VERSI MODIFIKASI UNTUK LINUX/UNIVERSAL ---

import cv2

def find_available_cameras(max_cameras_to_check=10):
    """
    Mendeteksi dan menampilkan semua indeks kamera yang tersedia
    menggunakan backend default OpenCV yang sesuai untuk sistem operasi.
    """
    print("--- Memulai Pengecekan Kamera ---")
    
    available_indices = []
    for i in range(max_cameras_to_check):
        # Menggunakan backend default, tanpa DSHOW atau MSMF
        cap = cv2.VideoCapture(i) 
        
        if cap and cap.isOpened():
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"  ✅ Kamera ditemukan di indeks: {i} (Resolusi: {int(width)}x{int(height)})")
            available_indices.append(i)
            # Penting: Lepaskan kamera agar bisa digunakan oleh program lain
            cap.release()
        else:
            print(f"  -- Tidak ada kamera di indeks: {i}")

    print("\n--- Ringkasan Hasil Pengecekan ---")

    if not available_indices:
        print("\n❌ KRITIS: Tidak ada kamera yang terdeteksi oleh OpenCV.")
        print("   1. Pastikan kamera terhubung dengan baik.")
        print("   2. Jalankan 'ls /dev/video*' untuk memastikan OS mendeteksinya.")
        print("   3. Pastikan tidak ada aplikasi lain yang sedang menggunakan kamera.")
    else:
        print(f"👍 Indeks kamera yang tersedia di sistem Anda adalah: {available_indices}")
        print("   Silakan perbarui file 'config.json' Anda dengan indeks ini.")

    print("\n------------------------------------")


if __name__ == "__main__":
    find_available_cameras()