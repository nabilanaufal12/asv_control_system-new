#include <Wire.h>
// #include <TinyGPS++.h> // Dihapus
#include <SparkFun_Ublox_Arduino_Library.h> // Ditambahkan
#include <ESP32Servo.h>
#include <Preferences.h>

// ---------------- GPS (Diganti ke U-Blox 10Hz) ----------------
SFE_UBLOX_GPS myGPS; // Ditambahkan
HardwareSerial gpsSerial(2); // Pindah ke Serial 2
#define GPS_BAUD_RATE 9600 // Sesuaikan dengan setting U-Blox Anda (9600 atau 115200)
#define GPS_RX_PIN 16 // Pin default Serial 2 (RX)
#define GPS_TX_PIN 17 // Pin default Serial 2 (TX)
// Catatan: Pin GPS_RX lama (15) tidak digunakan lagi.

// ---------------- CMPS12 ----------------
#define CMPS12_ADDRESS 0x60
#define ANGLE_16BIT_REGISTER 2

// ---------------- Servo dan ESC ----------------
Servo rudderServo;
Servo motorESC;
Servo auxOutput;     // DARI KODE MANUAL: Tambahkan servo untuk output tambahan

// ---------------- PID ----------------
double Kp = 2.0, Ki = 0.0, Kd = 0.5;
double error, lastError = 0, integral = 0;

// ---------------- Waypoint ----------------
#define MAX_DATA 15 // Maksimal 15 titik data
Preferences preferences;

float latitudes[MAX_DATA];
float longitudes[MAX_DATA];
int dataIndex = 0;
int counter = 0; // Counter untuk navigasi waypoint

bool captureTriggered = false;
bool wasInCaptureMode = false;
bool wasInSaveMode = false;

// --- [FIX] VARIABEL BARU UNTUK KONTROL DARI JETSON ---
char serialCommand = 'W'; // Default ke mode Waypoint ('W')
int ai_servo_val = 90;    // Nilai default untuk servo
int ai_motor_val = 1500;  // Nilai default untuk motor (netral)

// ---------------- Haversine ----------------
#define R 6371000.0
double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// ---------------- Bearing ----------------
double bearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double brng = atan2(y, x);
  return fmod((degrees(brng) + 360.0), 360.0);
}

// ---------------- Baca heading CMPS12 ----------------
float readCompass() {
  Wire.beginTransmission(CMPS12_ADDRESS);
  Wire.write(ANGLE_16BIT_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(CMPS12_ADDRESS, 2);
  if (Wire.available() == 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    unsigned int angle16 = (highByte << 8) | lowByte;
    return angle16 / 10.0;
  }
  return -1;
}

// ---------------- PID untuk servo ----------------
int PID_servo(double setpoint, double input) {
  error = input - setpoint;   // ? sudah dibalik biar logika servo benar

  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  integral += error;
  double derivative = error - lastError;
  lastError = error;

  double output = Kp * error + Ki * integral + Kd * derivative;
  int servoPos = 90 + output;

  if (servoPos > 180) servoPos = 180;
  if (servoPos < 0) servoPos = 0;

  return servoPos;
}

// ---------------- PPM INPUT ----------------
#define PPM_PIN 4
#define CHANNELS 10 // DIUBAH: Jumlah channel PPM disesuaikan menjadi 10 untuk membaca CH8
volatile int ppm[CHANNELS];
volatile byte ppmCounter = 0;
volatile unsigned long lastMicros = 0;

void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 3000) {
    ppmCounter = 0;
  } else {
    if (ppmCounter < CHANNELS) {
      ppm[ppmCounter] = diff;
      ppmCounter++;
    }
  }
}

int readChannel(byte ch, int minVal = 1000, int maxVal = 2000, int defaultVal = 1500) {
  if (ch < CHANNELS) {
    int val = ppm[ch];
    if (val >= 800 && val <= 2200) return val;
  }
  return defaultVal;
}

// ---------------- Fungsi Manajemen Data GPS ----------------
void saveDataToMemory() {
  preferences.begin("gps-data", false);
  preferences.putUInt("dataCount", dataIndex);
  for (int i = 0; i < dataIndex; i++) {
    String latKey = "lat" + String(i);
    String lngKey = "lng" + String(i);
    preferences.putFloat(latKey.c_str(), latitudes[i]);
    preferences.putFloat(lngKey.c_str(), longitudes[i]);
  }
  preferences.end();
}

void loadDataFromMemory() {
  preferences.begin("gps-data", true);
  dataIndex = preferences.getUInt("dataCount", 0);
  if (dataIndex > MAX_DATA) {
    dataIndex = MAX_DATA;
  }
  for (int i = 0; i < dataIndex; i++) {
    String latKey = "lat" + String(i);
    String lngKey = "lng" + String(i);
    latitudes[i] = preferences.getFloat(latKey.c_str(), 0.0);
    longitudes[i] = preferences.getFloat(lngKey.c_str(), 0.0);
  }
  preferences.end();
}

void clearAllData() {
  preferences.begin("gps-data", false);
  preferences.clear();
  preferences.end();
  dataIndex = 0;
  Serial.println("? Semua data lama telah dihapus.");
}

void displayAllData() {
  if (dataIndex > 0) {
    Serial.println("? DATA KOORDINAT TERSIMPAN:");
    Serial.println("==========================================");
    for (int i = 0; i < dataIndex; i++) {
      Serial.print("Titik ");
      if (i < 9) Serial.print("0");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(latitudes[i], 6);
      Serial.print(", ");
      Serial.println(longitudes[i], 6);
    }
    Serial.println("==========================================");
    Serial.print("Total: ");
    Serial.print(dataIndex);
    Serial.print("/");
    Serial.print(MAX_DATA);
    Serial.println(" titik");
  } else {
    Serial.println("? Tidak ada data koordinat yang tersimpan.");
  }
}

// ---------------- MODE FLAG ----------------
bool isManual = true;

// --- [FIX] FUNGSI BARU UNTUK MEMBACA PERINTAH SERIAL ---
void checkSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Hapus spasi atau karakter tak terlihat

    // Contoh format data dari Jetson: "A,95,1650" atau "W"
    
    // Ambil karakter pertama sebagai perintah
    if (input.length() > 0) {
      serialCommand = input.charAt(0);
    }

    // Jika perintahnya adalah 'A', parse nilai servo dan motor
    if (serialCommand == 'A') {
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > 0) {
        String servoStr = input.substring(firstComma + 1, secondComma);
        String motorStr = input.substring(secondComma + 1);
        ai_servo_val = servoStr.toInt();
        ai_motor_val = motorStr.toInt();
      }
    }
    // Jika perintah lain (misal 'W'), tidak perlu parsing lagi
  }
}


void setup() {
  Serial.begin(115200);
  
  // --- Inisialisasi GPS U-Blox (Baru) ---
  Serial.println("Mencoba koneksi ke GPS U-Blox...");
  // Mulai Serial GPS (Serial 2)
  // PASTIKAN KABEL GPS TERPASANG KE PIN 16 (RX) dan 17 (TX)
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (myGPS.begin(gpsSerial)) {
    Serial.println("Koneksi GPS berhasil!");
  } else {
    Serial.println("Gagal koneksi ke GPS. Cek kabel & baud rate.");
    // Tidak berhenti, tapi navigasi tidak akan jalan
  }

  // --- Konfigurasi GPS untuk UBX 10Hz ---
  myGPS.setUART1Output(COM_TYPE_UBX); // Matikan NMEA
  myGPS.setNavigationFrequency(10); // 10Hz
  myGPS.setAutoPVT(true); // Aktifkan pesan NAV-PVT

  uint8_t navFreq = myGPS.getNavigationFrequency();
  if (navFreq == 10) {
    Serial.println("GPS berhasil dikonfigurasi ke 10Hz.");
  } else {
    Serial.print("Konfigurasi 10Hz GAGAL! Frekuensi saat ini: ");
    Serial.print(navFreq);
    Serial.println(" Hz");
  }
  // --- Akhir Inisialisasi GPS U-Blox ---

  Wire.begin(21, 22);

  rudderServo.attach(18);
  motorESC.attach(19);
  auxOutput.attach(23); // DARI KODE MANUAL: Inisialisasi pin untuk auxOutput

  rudderServo.write(90);
  motorESC.writeMicroseconds(1500);
  auxOutput.writeMicroseconds(1500); // DARI KODE MANUAL: Set auxOutput ke posisi netral

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  // Muat data yang tersimpan dari memori
  loadDataFromMemory();

  Serial.println("");
  Serial.println("? GPS + WAYPOINT SYSTEM (INTEGRATED)");
  Serial.println("================================");
  Serial.print("Data tersimpan: ");
  Serial.print(dataIndex);
  Serial.print("/");
  Serial.print(MAX_DATA);
  Serial.println(" titik");
  Serial.println("================================");
}

// --- Variabel Global Telemetri (Ditambahkan untuk U-Blox) ---
float heading = 0.0; // Variabel heading sudah ada dari kompas
double lat = 0.0, lon = 0.0;
double speed = 0.0; // km/jam
int sats = 0;
// ---------------------------------

void loop() {
  // --- 1. Baca GPS U-Blox (Baru) ---
  // Cek data GPS baru dari U-Blox (non-blocking)
  // Fungsi getPVT() otomatis mem-parsing data jika ada
  if (myGPS.getPVT()) {
    // Ada data baru, update variabel global
    uint8_t fixType = myGPS.getFixType();

    if (fixType > 0) { // Cek jika ada fix (mis. 2D atau 3D)
        lat = myGPS.getLatitude() / 10000000.0;
        lon = myGPS.getLongitude() / 10000000.0;
        // Konversi mm/s (U-Blox) ke km/h
        speed = myGPS.getGroundSpeed() / 1000.0 * 3.6;
        sats = myGPS.getSIV(); // Satellites In View
    } else {
        // Tidak ada fix, reset data
        lat = 0.0;
        lon = 0.0;
        speed = 0.0;
        sats = 0;
    }
  }
  // Variabel lat, lon, speed, sats global sekarang diperbarui
  
  // --- [FIX] PANGGIL FUNGSI PEMBACA SERIAL DI SETIAP LOOP ---
  checkSerialInput();

  int ch5 = readChannel(4); // Mode Selector (Manual/Auto)
  int ch6 = readChannel(5); // Data Capture/Display/Save

  // ----------------- MANUAL MODE -----------------
  if (ch5 < 1500) {
    if (!isManual) {
      Serial.println("Switching to MANUAL...");
      rudderServo.write(90);
      motorESC.writeMicroseconds(1500);
      auxOutput.writeMicroseconds(1500); // Pastikan aux netral saat ganti mode
      isManual = true;
      wasInCaptureMode = false;
      wasInSaveMode = false;
    }

    // Kontrol Manual (Rudder)
    int ch1 = readChannel(0);
    int servoPos = map(ch1, 1000, 2000, 0, 180);
    rudderServo.write(servoPos);

    // Kontrol Manual (ESC/Throttle)
    int ch3 = readChannel(2);
    motorESC.writeMicroseconds(ch3);

    // DITAMBAHKAN: Kontrol Manual (Auxiliary CH8)
    int ch8 = readChannel(7);   // CH8 berada di index 7
    auxOutput.writeMicroseconds(ch8);
    
    // Kontrol waypoint (rekam / simpan) di mode MANUAL
    if (ch6 >= 1400 && ch6 <= 1600) {
      if (!wasInCaptureMode) {
        Serial.println("? MODE REKAM: Siap merekam waypoint baru.");
        wasInCaptureMode = true;
        captureTriggered = false;
      }
    } else if (ch6 > 1900) { // Rekam waypoint
      if (wasInCaptureMode && !captureTriggered) {
        if (wasInSaveMode) {
          clearAllData();
          wasInSaveMode = false;
        }

        if (dataIndex >= MAX_DATA) {
          Serial.println("? Memori penuh. Tidak bisa menambah titik lagi.");
        } else {
          // --- DIGANTI: Menggunakan myGPS.getFixType() dan variabel global lat/lon ---
          if (myGPS.getFixType() > 0) { // Cek jika GPS valid
            latitudes[dataIndex] = lat;  // Gunakan global 'lat'
            longitudes[dataIndex] = lon; // Gunakan global 'lon'
            dataIndex++;
            saveDataToMemory();

            Serial.println("? Titik ke-" + String(dataIndex) + " direkam.");
          } else {
            Serial.println("? GPS belum lock. Tidak dapat menambah data.");
          }
        }
        captureTriggered = true;
      }
      wasInCaptureMode = false;
    } else if (ch6 < 1100) { // Simpan data
      if (!wasInSaveMode) {
        saveDataToMemory();
        Serial.println("? Semua waypoint tersimpan.");
        displayAllData();
        wasInSaveMode = true;
      }
      wasInCaptureMode = false;
    }
  }

  // ----------------- AUTO MODE -----------------
    else {
    if (isManual) {
      Serial.println("Switching to AUTO...");
      rudderServo.write(90);
      motorESC.writeMicroseconds(1500);
      isManual = false;
      counter = 0;
    }

    // --- MODIFIKASI: Logika Prioritas di Mode Auto ---
    if (serialCommand == 'A') {
      // PRIORITAS 1: Perintah AI dari Jetson
      rudderServo.write(ai_servo_val);
      motorESC.writeMicroseconds(ai_motor_val);
      
      // Kirim kembali data telemetri untuk konfirmasi
      Serial.print("DATA:AUTO,AI_MODE_ACTIVATED,SERVO:");
      Serial.print(ai_servo_val);
      Serial.print(",MOTOR:");
      Serial.println(ai_motor_val);
    } 
    else if (serialCommand == 'W') {
      // PRIORITAS 2: Jalankan logika waypoint internal jika tidak ada perintah AI
      
      // --- DIGANTI: Menggunakan myGPS.getFixType() ---
      if (dataIndex > 0 && myGPS.getFixType() > 0) {
        if (counter >= dataIndex) {
          // Semua waypoint selesai ? berhenti
          rudderServo.write(90);
          motorESC.writeMicroseconds(1000); // Ubah ke 1000 atau 1500 (netral) sesuai kebutuhan
          Serial.println("DATA:AUTO,WAYPOINT_COMPLETED");
        } else {
          // --- DIHAPUS: Deklarasi lokal lat, lon, speed, sats ---
          // Variabel global (lat, lon, speed, sats) dari atas loop akan digunakan
          
          double targetLat = latitudes[counter];
          double targetLon = longitudes[counter];
          double dist = haversine(lat, lon, targetLat, targetLon); // Menggunakan global 'lat' & 'lon'
          double targetBearing = bearing(lat, lon, targetLat, targetLon); // Menggunakan global 'lat' & 'lon'
          heading = readCompass(); // 'heading' sudah global/di-update
          double errorHeading = targetBearing - heading;
          if (errorHeading > 180) errorHeading -= 360;
          if (errorHeading < -180) errorHeading += 360;
          int servoPos = PID_servo(targetBearing, heading);
          rudderServo.write(servoPos);
    
          int motorSpeed = 1300;
          if (dist > 1.0) motorSpeed = 1300;
          else if (dist > 1.0) motorSpeed = 1300;
          else motorSpeed = 1300;
          motorESC.writeMicroseconds(motorSpeed);
          
          if (dist < 1.75) {
            counter++;
          }
    
          Serial.print("DATA:AUTO,WAYPOINT,");
          Serial.print(counter + 1);
          Serial.print(",");
          Serial.print(dist);
          Serial.print(",");
          Serial.print(targetBearing);
          Serial.print(",");
          Serial.print(heading);
          Serial.print(",");
          Serial.print(errorHeading);
          Serial.print(",");
          Serial.print(servoPos);
          Serial.print(",");
          Serial.print(motorSpeed);
          Serial.print(",");
          Serial.print(speed); // Menggunakan global 'speed'
          Serial.print(",");
          Serial.println(sats); // Menggunakan global 'sats'
        }
      } else {
        rudderServo.write(90);
        motorESC.writeMicroseconds(1500);
        if (dataIndex == 0) {
          Serial.println("DATA:AUTO,NO_WAYPOINTS");
        } else {
          Serial.println("DATA:AUTO,GPS_INVALID"); // Sekarang juga menangani myGPS.getFixType() == 0
        }
      }
    }
  }

  delay(50); // Kurangi delay agar lebih responsif
}