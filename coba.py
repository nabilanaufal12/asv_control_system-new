import serial
import time
import sys
import threading
from serial.serialutil import SerialException
from pynput.keyboard import Key, Listener

# Pastikan Anda menginstal pynput: pip install pynput
# Di terminal, jalankan skrip ini dengan perintah: python laptop_monitor_v2.py

is_a_pressed = False
last_command_sent = None

def clear_screen():
    """Fungsi untuk membersihkan layar konsol."""
    import os
    os.system('cls' if os.name == 'nt' else 'clear')

def on_press(key):
    """
    Fungsi yang dipanggil ketika sebuah tombol ditekan.
    """
    global is_a_pressed
    try:
        if key.char == 'a' and not is_a_pressed:
            is_a_pressed = True
    except AttributeError:
        # Menangani tombol non-karakter (seperti Esc)
        pass

def on_release(key):
    """
    Fungsi yang dipanggil ketika sebuah tombol dilepas.
    """
    global is_a_pressed
    try:
        if key.char == 'a':
            is_a_pressed = False
    except AttributeError:
        pass
    
    # Hentikan listener jika tombol Esc ditekan
    if key == Key.esc:
        print("\nMenutup listener keyboard...")
        return False

def main():
    """
    Skrip utama untuk membaca data dari Serial Port dan mengirimkan perintah.
    """
    global last_command_sent

    port_name = input("Masukkan nama Serial Port (contoh: COM3 atau /dev/ttyUSB0): ")
    baud_rate = 115200

    try:
        ser = serial.Serial(port_name, baud_rate, timeout=0.1)
        print(f"\nMenghubungkan ke port {port_name}...")
        time.sleep(2) # Beri waktu agar koneksi stabil

        # Mulai listener keyboard dalam thread terpisah
        listener = Listener(on_press=on_press, on_release=on_release)
        listener.start()

        clear_screen()
        print("Menunggu data dari ESP32...")
        print("Ketik 'A' dan tahan untuk mengaktifkan mode AI.")
        print("Lepas 'A' untuk kembali ke mode Waypoint.")
        print("Tekan 'Esc' untuk keluar.")

        while True:
            # Mengirim perintah berdasarkan status tombol 'A'
            command_to_send = 'A' if is_a_pressed else 'W'
            if command_to_send != last_command_sent:
                ser.write(command_to_send.encode('utf-8'))
                last_command_sent = command_to_send
                if command_to_send == 'A':
                    print("✅ Mengirim 'A': Masuk Mode AI")
                else:
                    print("✅ Mengirim 'W': Kembali ke Waypoint")

            # Baca satu baris dari serial port
            line = ser.readline().decode('utf-8').strip()

            if line:
                if line.startswith("DATA:"):
                    data_string = line[5:]
                    parts = data_string.split(',')
                    mode = parts[0]
                    
                    if mode == "AUTO":
                        if parts[1] == "AI_MODE_ACTIVATED":
                            clear_screen()
                            print("====================================")
                            print("       MODE: AUTO [PENGHINDARAN RINTANGAN AI]")
                            print("====================================")
                            print("  ⚠️ KONTROL DIAMBIL ALIH OLEH LAPTOP.")
                        elif parts[1] == "RESUMED_FROM_AI":
                            clear_screen()
                            print("====================================")
                            print("       MODE: AUTO [KEMBALI KE WAYPOINT]")
                            print("====================================")
                            print("  ✅ Rintangan hilang. Melanjutkan navigasi waypoint.")
                        
                        elif len(parts) == 10:
                            clear_screen()
                            _, wp, dist, target_bearing, heading, error, servo, motor, speed, sats = parts
                            print("====================================")
                            print("       MODE: AUTO [NAVIGASI OTOMATIS]")
                            print("====================================")
                            print(f"  ● Waypoint: {wp}")
                            print(f"  ● Jarak ke WP: {dist} m")
                            print(f"  ● Target Heading: {target_bearing}°")
                            print(f"  ● Heading Saat Ini: {heading}°")
                            print(f"  ● Error Heading: {error}°")
                            print(f"  ● Servo: {servo}°")
                            print(f"  ● Motor: {motor}μs")
                            print(f"  ● Kecepatan: {speed} km/h")
                            print(f"  ● Satelit: {sats}")

                    elif mode == "MANUAL":
                        if len(parts) == 8:
                            clear_screen()
                            _, lat, lon, heading, servo, motor, speed, sats = parts
                            print("====================================")
                            print("       MODE: MANUAL [KONTROL VIA RC]")
                            print("====================================")
                            print(f"  ● GPS: {lat}, {lon}")
                            print(f"  ● Heading: {heading}°")
                            print(f"  ● Servo: {servo}°")
                            print(f"  ● Motor: {motor}μs")
                            print(f"  ● Kecepatan: {speed} km/h")
                            print(f"  ● Satelit: {sats}")
                        else:
                            print("GPS belum valid. Mohon tunggu...")
                    
                    else:
                        if parts[1] == "STOPPED":
                            clear_screen()
                            print("====================================")
                            print("       MODE: AUTO [WAYPOINTS SELESAI]")
                            print("====================================")
                            print("  🛑 KAPAL BERHENTI. SEMUA WAYPOINT TELAH DICAPAI.")
                        elif parts[1] == "NO_WAYPOINTS":
                            clear_screen()
                            print("====================================")
                            print("       MODE: AUTO [TIDAK ADA WAYPOINT]")
                            print("====================================")
                            print("  ⚠ TIDAK ADA WAYPOINT TERSIMPAN. KAPAL BERHENTI.")
                        elif parts[1] == "GPS_INVALID":
                            clear_screen()
                            print("====================================")
                            print("       MODE: AUTO [GPS TIDAK VALID]")
                            print("====================================")
                            print("  ❌ GPS belum lock. Menunggu sinyal...")
                        else:
                            print("Kesalahan parsing data dari ESP32.")
            
            # Periksa apakah listener masih berjalan
            if not listener.is_alive():
                break

            time.sleep(0.1) # Tunda sebentar untuk menghindari penggunaan CPU berlebih

    except SerialException as e:
        print(f"Error: Tidak bisa terhubung ke serial port {port_name}.")
        print("Pastikan ESP32 terhubung dan nama port sudah benar.")
        print("Error: ", e)
    except KeyboardInterrupt:
        print("\nSkrip dihentikan.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Koneksi serial ditutup.")
        if 'listener' in locals() and listener.is_alive():
            listener.stop()
            
if __name__ == "__main__":
    main()