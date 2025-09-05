# check_cameras.py
# Skrip diagnostik untuk menemukan kamera yang tersedia dengan berbagai backend OpenCV.

import cv2


def find_available_cameras(max_cameras_to_check=5):
    """
    Mendeteksi dan menampilkan semua indeks kamera yang tersedia,
    mencoba dengan backend DSHOW dan MSMF untuk kompatibilitas Windows.
    """
    print("--- Memulai Pengecekan Kamera ---")

    # Backend yang akan kita uji, keduanya umum di Windows
    backends_to_test = {"DSHOW": cv2.CAP_DSHOW, "MSMF": cv2.CAP_MSMF}

    found_cameras = {}

    for name, backend in backends_to_test.items():
        print(f"\n🔎 Menguji dengan backend: {name}")
        available_for_backend = []
        for i in range(max_cameras_to_check):
            # Mencoba membuka kamera dengan backend spesifik
            cap = cv2.VideoCapture(i, backend)
            if cap and cap.isOpened():
                # Membaca properti kamera untuk verifikasi lebih lanjut
                width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                print(
                    f"  ✅ Kamera ditemukan di indeks: {i} (Resolusi: {int(width)}x{int(height)})"
                )
                available_for_backend.append(i)
                # Penting: Lepaskan kamera agar bisa digunakan oleh program lain
                cap.release()
            else:
                print(f"  -- Tidak ada kamera di indeks: {i}")

        found_cameras[name] = available_for_backend

    print("\n--- Ringkasan Hasil Pengecekan ---")

    dshow_cameras = found_cameras["DSHOW"]
    msmf_cameras = found_cameras["MSMF"]

    if not dshow_cameras and not msmf_cameras:
        print("\n❌ KRITIS: Tidak ada kamera yang terdeteksi oleh OpenCV.")
        print("   Pastikan kamera terhubung dengan baik dan driver sudah terinstal.")
        print(
            "   Pastikan juga tidak ada aplikasi lain (Zoom, OBS, dll.) yang sedang menggunakan kamera."
        )
    else:
        print(f"Hasil DSHOW: {dshow_cameras if dshow_cameras else 'Tidak ada'}")
        print(f"Hasil MSMF:  {msmf_cameras if msmf_cameras else 'Tidak ada'}")

        print("\n--- Rekomendasi ---")
        if msmf_cameras:
            print(
                f"👍 Gunakan salah satu dari indeks berikut di config.json Anda: {msmf_cameras}"
            )
            print(
                "   Jika aplikasi utama masih gagal, modifikasi file vision_service.py"
            )
            print(
                "   untuk menggunakan 'cv2.CAP_MSMF' saat memanggil cv2.VideoCapture."
            )
        elif dshow_cameras:
            print(
                f"👍 Gunakan salah satu dari indeks berikut di config.json Anda: {dshow_cameras}"
            )
            print("   Backend DSHOW tampaknya berfungsi dengan baik.")

        print("\n------------------------------------")


if __name__ == "__main__":
    find_available_cameras()
