import serial.tools.list_ports

def diagnose_serial_ports():
    """
    Mendeteksi dan menampilkan informasi detail tentang semua port serial
    yang terhubung ke sistem.
    """
    print("--- Memulai Diagnosis Port Serial ---")
    
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("\n\033[91m[HASIL] KRITIS: Tidak ada port serial yang terdeteksi sama sekali.\033[0m")
        print("  - Pastikan ESP32 Anda terhubung dengan kuat ke port USB Jetson.")
        print("  - Coba ganti kabel USB atau port USB.")
        print("  - Pastikan ESP32 menyala (biasanya ada lampu LED power).")
        return

    print(f"\n\033[92m[HASIL] Ditemukan {len(ports)} port serial:\033[0m")
    
    for port in ports:
        print("\n-----------------------------------------")
        print(f"  \033[1mDevice:\033[0m      {port.device}")
        print(f"  \033[1mDescription:\033[0m {port.description}")
        print(f"  \033[1mHardware ID:\033[0m {port.hwid}")
    
    print("\n-----------------------------------------")
    print("\n\033[93m[REKOMENDASI]\033[0m")
    print("1. Bandingkan 'Description' di atas dengan daftar 'auto_connect_descriptors' di 'config/config.json' Anda.")
    print("2. Jika ada deskripsi yang cocok tapi koneksi 'AUTO' gagal, pastikan Anda sudah memperbaiki izin (permissions).")
    print("3. Jika tidak ada deskripsi yang cocok, tambahkan deskripsi unik dari perangkat Anda ke dalam daftar di config.json.")
    print("4. Sebagai alternatif, Anda bisa memilih 'Device' secara manual (misal: /dev/ttyUSB0) di GUI daripada menggunakan 'AUTO'.")
    print("-----------------------------------------")


if __name__ == "__main__":
    diagnose_serial_ports()