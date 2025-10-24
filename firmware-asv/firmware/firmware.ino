#include <Wire.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ---------------- GPS ----------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
// Sesuaikan dengan pin GPS
#define GPS_RX 15
#define GPS_TX 17

// ---------------- CMPS12 ----------------
#define CMPS12_ADDRESS 0x60
#define ANGLE_16BIT_REGISTER 2

// ---------------- Servo dan ESC ----------------
Servo rudderServo;
Servo motorESC;
Servo auxOutput; // DARI KODE MANUAL: Tambahkan servo untuk output tambahan

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
  return -1; // Return -1 jika gagal membaca
}

// ---------------- PID untuk servo ----------------
int PID_servo(double setpoint, double input) {
  error = input - setpoint; // Logika servo sudah dibalik

  // Wrap error to +/- 180 degrees
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  integral += error;
  double derivative = error - lastError;
  lastError = error;

  double output = Kp * error + Ki * integral + Kd * derivative;

  int servoPos = 90 + output; // Assume 90 is center

  // Clamp servo position
  if (servoPos > 180) servoPos = 180;
  if (servoPos < 0) servoPos = 0;

  return servoPos;
}

// ---------------- PPM INPUT ----------------
#define PPM_PIN 4
#define CHANNELS 10 // Jumlah channel PPM
volatile int ppm[CHANNELS];
volatile byte ppmCounter = 0;
volatile unsigned long lastMicros = 0;

void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 3000) { // Sync pulse detected
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
    // Basic validation for PPM values
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
  Serial.println("? Data waypoint disimpan ke memori.");
}

void loadDataFromMemory() {
  preferences.begin("gps-data", true); // Read-only mode
  dataIndex = preferences.getUInt("dataCount", 0);
  if (dataIndex > MAX_DATA) {
    Serial.println("? Peringatan: Data tersimpan melebihi MAX_DATA. Dibatasi.");
    dataIndex = MAX_DATA;
  }
  for (int i = 0; i < dataIndex; i++) {
    String latKey = "lat" + String(i);
    String lngKey = "lng" + String(i);
    latitudes[i] = preferences.getFloat(latKey.c_str(), 0.0);
    longitudes[i] = preferences.getFloat(lngKey.c_str(), 0.0);
  }
  preferences.end();
  Serial.print("? Data waypoint dimuat dari memori: ");
  Serial.print(dataIndex);
  Serial.println(" titik.");
}

void clearAllData() {
  preferences.begin("gps-data", false);
  preferences.clear();
  preferences.end();
  dataIndex = 0;
  Serial.println("? Semua data waypoint di memori telah dihapus.");
}

void displayAllData() {
  if (dataIndex > 0) {
    Serial.println("? DATA KOORDINAT TERSIMPAN:");
    Serial.println("==========================================");
    for (int i = 0; i < dataIndex; i++) {
      Serial.print("Titik ");
      if (i < 9) Serial.print("0"); // Leading zero for single digits
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

// --- Fungsi untuk Membaca Perintah Serial dari Jetson ---
void checkSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Hapus spasi atau karakter tak terlihat

    if (input.length() > 0) {
      serialCommand = input.charAt(0); // Ambil karakter pertama sebagai perintah

      // Jika perintahnya adalah 'A', parse nilai servo dan motor
      if (serialCommand == 'A') {
        int firstComma = input.indexOf(',');
        int secondComma = input.indexOf(',', firstComma + 1);

        if (firstComma > 0 && secondComma > 0) {
          String servoStr = input.substring(firstComma + 1, secondComma);
          String motorStr = input.substring(secondComma + 1);
          ai_servo_val = servoStr.toInt();
          ai_motor_val = motorStr.toInt();
        } else {
           Serial.println("? Peringatan: Format perintah 'A' tidak valid.");
           serialCommand = 'W'; // Kembali ke default jika format salah
        }
      }
      // Jika perintah lain (misal 'W'), tidak perlu parsing lagi
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin(21, 22); // SDA, SCL pins for CMPS12

  rudderServo.attach(18);
  motorESC.attach(19);
  auxOutput.attach(23); // Pin untuk output tambahan (CH8)

  // Initialize actuators to neutral positions
  rudderServo.write(90);
  motorESC.writeMicroseconds(1500);
  auxOutput.writeMicroseconds(1500);

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  loadDataFromMemory(); // Muat data waypoint saat startup

  Serial.println("");
  Serial.println("======================================");
  Serial.println("?   ASV NAVANTARA - ESP32 Firmware   ?");
  Serial.println("======================================");
  Serial.print("? Data tersimpan: ");
  Serial.print(dataIndex);
  Serial.print("/");
  Serial.print(MAX_DATA);
  Serial.println(" titik");
  Serial.println("======================================");
}

// --- VARIABEL GLOBAL UNTUK TELEMETRI ---
float heading = 0.0;
double lat = 0.0, lon = 0.0, speed = 0.0; // speed in kmph
int sats = 0;
// ----------------------------------------

// ================= LOOP =================
void loop() {
  // --- Baca Sensor di Awal Loop ---
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    speed = gps.speed.kmph(); // Simpan kecepatan dalam km/jam
  } else {
    // Reset jika GPS tidak valid untuk sementara
    lat = 0.0;
    lon = 0.0;
    speed = 0.0;
  }
  if (gps.satellites.isValid()) {
    sats = gps.satellites.value();
  } else {
    sats = 0;
  }
  heading = readCompass();
  if (heading == -1) {
    // Handle error pembacaan kompas jika perlu
    // Serial.println("? Peringatan: Gagal membaca kompas.");
  }

  checkSerialInput(); // Baca perintah dari Jetson

  // Baca input dari Remote Control (RC)
  int ch1 = readChannel(0); // Rudder (Kanan/Kiri)
  int ch3 = readChannel(2); // Throttle (Maju/Mundur)
  int ch5 = readChannel(4); // Mode Selector (Manual/Auto)
  int ch6 = readChannel(5); // Waypoint Capture/Save trigger
  int ch8 = readChannel(7); // Auxiliary Output

  // ----------------- MANUAL MODE (RC Override) -----------------
  if (ch5 < 1500) {
    if (!isManual) {
      Serial.println("? Beralih ke MODE MANUAL (RC)...");
      rudderServo.write(90);             // Set netral saat ganti mode
      motorESC.writeMicroseconds(1500);
      auxOutput.writeMicroseconds(1500);
      isManual = true;
      wasInCaptureMode = false;
      wasInSaveMode = false;
    }

    // Kontrol Manual Langsung dari RC
    int servoPosManual = map(ch1, 1000, 2000, 0, 180);
    rudderServo.write(servoPosManual);
    motorESC.writeMicroseconds(ch3);
    auxOutput.writeMicroseconds(ch8);

    // --- Logika Penyimpanan Waypoint di Mode MANUAL ---
    if (ch6 >= 1400 && ch6 <= 1600) { // Mode Rekam Aktif
      if (!wasInCaptureMode) {
        Serial.println("? MODE REKAM AKTIF: Siap merekam waypoint baru via RC.");
        wasInCaptureMode = true;
        captureTriggered = false; // Reset trigger
      }
      wasInSaveMode = false; // Keluar dari mode save jika masuk mode rekam
    } else if (ch6 > 1900) { // Tombol Rekam Ditekan
      if (wasInCaptureMode && !captureTriggered) {
        if (wasInSaveMode) { // Jika sebelumnya di mode save, hapus dulu
          clearAllData();
          wasInSaveMode = false;
        }

        if (dataIndex >= MAX_DATA) {
          Serial.println("? Memori waypoint penuh. Tidak bisa menambah titik.");
        } else {
          if (gps.location.isValid()) {
            latitudes[dataIndex] = lat; // Gunakan lat global yang sudah dibaca
            longitudes[dataIndex] = lon; // Gunakan lon global yang sudah dibaca
            dataIndex++;
            saveDataToMemory(); // Langsung simpan setiap titik
            Serial.print("? Titik ke-");
            Serial.print(dataIndex);
            Serial.println(" direkam.");
          } else {
            Serial.println("? GPS belum valid. Tidak dapat merekam waypoint.");
          }
        }
        captureTriggered = true; // Hanya rekam sekali per tekanan
      }
      // Keluar dari mode rekam setelah tombol ditekan
      // wasInCaptureMode = false; // Opsional: tetap di mode rekam?
    } else if (ch6 < 1100) { // Tombol Simpan/Tampilkan Ditekan
      if (!wasInSaveMode) {
        // saveDataToMemory(); // Sudah disimpan per titik, tidak perlu lagi?
        Serial.println("? Menampilkan semua waypoint tersimpan...");
        displayAllData();
        wasInSaveMode = true;
      }
      wasInCaptureMode = false; // Keluar dari mode rekam jika masuk mode save
    } else { // Posisi netral CH6
      wasInCaptureMode = false;
      wasInSaveMode = false;
      captureTriggered = false; // Reset trigger saat kembali ke netral
    }

    // --- Kirim Telemetri Mode Manual ---
    Serial.print("DATA:MANUAL,");
    Serial.print(heading, 1); // 1 desimal untuk heading
    Serial.print(",");
    Serial.print(speed, 1);   // 1 desimal untuk speed (kmph)
    Serial.print(",");
    Serial.print(sats);
    Serial.print(",");
    Serial.print(servoPosManual); // Posisi servo dari RC
    Serial.print(",");
    Serial.println(ch3);          // Nilai motor dari RC

  }
  // ----------------- AUTO MODE (Dikontrol Jetson atau Waypoint Internal) -----------------
  else {
    if (isManual) {
      Serial.println("? Beralih ke MODE AUTO...");
      rudderServo.write(90); // Set netral saat ganti mode
      motorESC.writeMicroseconds(1500);
      isManual = false;
      counter = 0; // Reset counter waypoint
    }

    // --- Logika Prioritas di Mode Auto ---
    if (serialCommand == 'A') {
      // PRIORITAS 1: Perintah AI dari Jetson
      rudderServo.write(ai_servo_val);
      motorESC.writeMicroseconds(ai_motor_val);

      // Kirim konfirmasi kembali ke Jetson
      Serial.print("DATA:AUTO,AI_MODE_ACTIVATED,SERVO:");
      Serial.print(ai_servo_val);
      Serial.print(",MOTOR:");
      Serial.println(ai_motor_val);

    } else if (serialCommand == 'W') {
      // PRIORITAS 2: Navigasi Waypoint Internal ESP32
      if (dataIndex > 0 && gps.location.isValid()) { // Hanya jika ada waypoint & GPS valid
        if (counter >= dataIndex) { // Semua waypoint selesai
          rudderServo.write(90);
          motorESC.writeMicroseconds(1500); // Berhenti (atau 1000 jika perlu)
          Serial.println("DATA:AUTO,WAYPOINT_COMPLETED");

        } else { // Navigasi ke waypoint 'counter'
          double targetLat = latitudes[counter];
          double targetLon = longitudes[counter];

          double dist = haversine(lat, lon, targetLat, targetLon);
          double targetBearing = bearing(lat, lon, targetLat, targetLon);

          // Hitung error heading & jalankan PID
          // 'heading' sudah dibaca di awal loop
          double errorHeading = targetBearing - heading;
          if (errorHeading > 180) errorHeading -= 360;
          if (errorHeading < -180) errorHeading += 360;
          int servoPosAuto = PID_servo(targetBearing, heading);
          rudderServo.write(servoPosAuto);

          // Logika kecepatan motor sederhana berdasarkan jarak
          int motorSpeedAuto = 1700; // Kecepatan default
          if (dist < 3.0) motorSpeedAuto = 1600; // Pelan saat dekat
          if (dist < 1.0) motorSpeedAuto = 1500; // Netral saat sangat dekat
          motorESC.writeMicroseconds(motorSpeedAuto);

          // Pindah ke waypoint berikutnya jika sudah dekat
          if (dist < 1.5) {
            counter++;
            Serial.print("? Mencapai waypoint, lanjut ke titik ");
            Serial.println(counter + 1);
          }

          // Kirim Telemetri Navigasi Waypoint
          Serial.print("DATA:AUTO,WAYPOINT,");
          Serial.print(counter + 1); // Indeks waypoint target (mulai dari 1)
          Serial.print(",");
          Serial.print(dist, 1);
          Serial.print(",");
          Serial.print(targetBearing, 1);
          Serial.print(",");
          Serial.print(heading, 1);
          Serial.print(",");
          Serial.print(errorHeading, 1);
          Serial.print(",");
          Serial.print(servoPosAuto);
          Serial.print(",");
          Serial.print(motorSpeedAuto);
          Serial.print(",");
          Serial.print(speed, 1); // Kecepatan saat ini (kmph)
          Serial.print(",");
          Serial.println(sats);
        }
      } else { // Masalah di mode Waypoint (tidak ada data atau GPS tidak valid)
        rudderServo.write(90);
        motorESC.writeMicroseconds(1500); // Netral
        if (dataIndex == 0) {
          Serial.println("DATA:AUTO,NO_WAYPOINTS");
        } else {
          Serial.println("DATA:AUTO,GPS_INVALID");
        }
      }
    }
    // Jika ada serialCommand lain (selain 'A' atau 'W'), abaikan di mode AUTO
  }

  delay(100); // Jeda loop utama
}