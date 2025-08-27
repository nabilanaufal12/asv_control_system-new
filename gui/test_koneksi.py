import serial
import time

# Pastikan ini adalah port yang benar dari Device Manager
PORT = "COM3"
BAUDRATE = 115200

print(f"Mencoba membuka port {PORT}...")
try:
    # Coba buka koneksi
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"BERHASIL! Port {PORT} berhasil dibuka.")

    # Beri waktu ESP32 untuk siap
    time.sleep(2)

    print("Mencoba membaca data dari ESP32 selama 5 detik...")
    for i in range(5):
        line = ser.readline().decode("utf-8").strip()
        if line:
            print(f"Diterima: {line}")
        else:
            print("Menunggu data...")
        time.sleep(1)

    ser.close()
    print("Port berhasil ditutup.")

except serial.SerialException as e:
    # --- PERBAIKAN DI SINI ---
    print("\nGAGAL: Tidak bisa membuka port.")  # Menghapus f'' yang tidak perlu
    # -------------------------
    print(f"Error: {e}")
    print(
        "\nPastikan tidak ada aplikasi lain (seperti Serial Monitor Arduino) yang sedang menggunakan port ini."
    )

except Exception as e:
    print(f"GAGAL: Terjadi error yang tidak terduga: {e}")
