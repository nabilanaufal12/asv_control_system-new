# backend/app/services/serial_handler.py
# Kelas ini mengelola semua komunikasi serial ke mikrokontroler (ESP32).

import serial
import time

class SerialHandler:
    def __init__(self):
        self.ser = None
        self.is_connected = False
        print("[SerialHandler] Dibuat.")

    def connect(self, port, baudrate):
        """Mencoba terhubung ke port serial yang diberikan."""
        try:
            # Tutup koneksi lama jika ada
            if self.ser and self.ser.is_open:
                self.ser.close()

            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.is_connected = True
            print(f"[SerialHandler] Berhasil terhubung ke {port} dengan baudrate {baudrate}.")
            return True
        except serial.SerialException as e:
            self.is_connected = False
            print(f"[SerialHandler] Gagal terhubung ke port serial: {e}")
            return False

    def disconnect(self):
        """Menutup koneksi serial."""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.is_connected = False
        print("[SerialHandler] Koneksi serial ditutup.")

    def send_command(self, command_string):
        """Mengirim string perintah ke mikrokontroler."""
        if self.is_connected:
            try:
                # Tambahkan newline agar mudah dibaca di sisi Arduino/ESP32
                full_command = command_string + '\n'
                self.ser.write(full_command.encode('utf-8'))
                print(f"[SerialHandler] Mengirim: {command_string}")
            except serial.SerialException as e:
                print(f"[SerialHandler] Gagal mengirim data: {e}")
                self.disconnect()
        else:
            # Mode simulasi jika tidak terhubung
            print(f"[SerialHandler (Simulasi)] Akan mengirim: {command_string}")

    def read_data(self):
        """Membaca satu baris data dari mikrokontroler (untuk nanti)."""
        if self.is_connected:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    return line
            except serial.SerialException as e:
                print(f"[SerialHandler] Gagal membaca data: {e}")
                self.disconnect()
        return None

