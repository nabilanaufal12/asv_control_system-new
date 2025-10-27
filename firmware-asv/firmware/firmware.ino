#include <Wire.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ---------------- GPS ----------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
#define GPS_RX 15
#define GPS_TX 17

// ---------------- CMPS12 ----------------
#define CMPS12_ADDRESS 0x60
#define ANGLE_16BIT_REGISTER 2

// ---------------- Servo dan ESC ----------------
Servo rudderServo;
Servo motorESC;
Servo auxOutput; // Output tambahan (CH8)

// ---------------- PID ----------------
double Kp = 2.0, Ki = 0.0, Kd = 0.5;
double error, lastError = 0, integral = 0;

// ---------------- Waypoint ----------------
#define MAX_DATA 15
Preferences preferences;
float latitudes[MAX_DATA];
float longitudes[MAX_DATA];
int dataIndex = 0;
int counter = 0; // Counter waypoint

bool captureTriggered = false;
bool wasInCaptureMode = false;
bool wasInSaveMode = false;

// --- Variabel Kontrol dari Jetson ---
char serialCommand = 'W'; // Default: Waypoint
int ai_servo_val = 90;
int ai_motor_val = 1500;

// ---------------- Haversine ----------------
#define R 6371000.0 // Radius Bumi (meter)
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
  return fmod((degrees(brng) + 360.0), 360.0); // Hasil 0-360 derajat
}

// ---------------- Baca Kompas CMPS12 ----------------
float readCompass() {
  Wire.beginTransmission(CMPS12_ADDRESS);
  Wire.write(ANGLE_16BIT_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(CMPS12_ADDRESS, 2);
  if (Wire.available() == 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    unsigned int angle16 = (highByte << 8) | lowByte;
    return angle16 / 10.0; // Hasil dalam derajat (0-359.9)
  }
  return -1; // Error
}

// ---------------- PID Servo ----------------
int PID_servo(double setpoint, double input) {
  error = input - setpoint; // Logika servo dibalik

  // Wrap error +/- 180
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  integral += error;
  // TODO: Tambahkan anti-windup jika perlu
  double derivative = error - lastError;
  lastError = error;

  double output = Kp * error + Ki * integral + Kd * derivative;
  int servoPos = 90 + output; // Asumsi 90 tengah

  // Batasi output servo
  servoPos = constrain(servoPos, 0, 180);
  return servoPos;
}

// ---------------- PPM Input RC ----------------
#define PPM_PIN 4
#define CHANNELS 10
volatile int ppm[CHANNELS];
volatile byte ppmCounter = 0;
volatile unsigned long lastMicros = 0;

void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 3000) { // Sync pulse (> 3ms)
    ppmCounter = 0;
  } else if (ppmCounter < CHANNELS) {
    ppm[ppmCounter++] = diff;
  }
}

int readChannel(byte ch, int minVal = 1000, int maxVal = 2000, int defaultVal = 1500) {
  if (ch < CHANNELS) {
    int val = ppm[ch];
    if (val >= 800 && val <= 2200) return val; // Validasi sederhana
  }
  return defaultVal;
}

// ---------------- Manajemen Waypoint (Preferences) ----------------
void saveDataToMemory() {
  preferences.begin("gps-data", false); // Buka untuk tulis
  preferences.putUInt("dataCount", dataIndex);
  for (int i = 0; i < dataIndex; i++) {
    String latKey = "lat" + String(i);
    String lngKey = "lng" + String(i);
    preferences.putFloat(latKey.c_str(), latitudes[i]);
    preferences.putFloat(lngKey.c_str(), longitudes[i]);
  }
  preferences.end();
  Serial.println("? Data waypoint disimpan ke NVS.");
}

void loadDataFromMemory() {
  preferences.begin("gps-data", true); // Buka untuk baca
  dataIndex = preferences.getUInt("dataCount", 0);
  if (dataIndex > MAX_DATA) {
    Serial.print("? Peringatan: Data di NVS > MAX_DATA. Dibatasi ke ");
    Serial.println(MAX_DATA);
    dataIndex = MAX_DATA;
  }
  for (int i = 0; i < dataIndex; i++) {
    String latKey = "lat" + String(i);
    String lngKey = "lng" + String(i);
    latitudes[i] = preferences.getFloat(latKey.c_str(), 0.0);
    longitudes[i] = preferences.getFloat(lngKey.c_str(), 0.0);
  }
  preferences.end();
  Serial.print("? Data waypoint dimuat dari NVS: ");
  Serial.print(dataIndex);
  Serial.println(" titik.");
}

void clearAllData() {
  preferences.begin("gps-data", false);
  preferences.clear();
  preferences.end();
  dataIndex = 0;
  Serial.println("? Semua data waypoint di NVS dihapus.");
}

void displayAllData() {
  if (dataIndex > 0) {
    Serial.println("? == Data Waypoint Tersimpan ==");
    for (int i = 0; i < dataIndex; i++) {
      Serial.print("  Titik ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(latitudes[i], 6);
      Serial.print(", ");
      Serial.println(longitudes[i], 6);
    }
    Serial.print("? Total: ");
    Serial.print(dataIndex);
    Serial.print("/");
    Serial.println(MAX_DATA);
    Serial.println("? =============================");
  } else {
    Serial.println("? Tidak ada waypoint tersimpan.");
  }
}

// ---------------- Mode Flag ----------------
bool isManual = true;

// --- Baca Perintah Serial dari Jetson ---
void checkSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      serialCommand = input.charAt(0); // 'A' atau 'W'

      if (serialCommand == 'A') {
        int firstComma = input.indexOf(',');
        int secondComma = input.indexOf(',', firstComma + 1);

        if (firstComma > 0 && secondComma > 0) {
          ai_servo_val = input.substring(firstComma + 1, secondComma).toInt();
          ai_motor_val = input.substring(secondComma + 1).toInt();
        } else {
           Serial.println("? Format Perintah 'A' salah, kembali ke 'W'.");
           serialCommand = 'W'; // Fallback
        }
      }
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin(21, 22); // SDA, SCL

  rudderServo.attach(18);
  motorESC.attach(19);
  auxOutput.attach(23); // CH8

  // Set netral
  rudderServo.write(90);
  motorESC.writeMicroseconds(1500);
  auxOutput.writeMicroseconds(1500);

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  loadDataFromMemory(); // Muat waypoints

  Serial.println("\n=================================");
  Serial.println("? ASV NAVANTARA - Firmware Siap ?");
  Serial.println("=================================");
  displayAllData(); // Tampilkan waypoint awal
  Serial.println("? Menunggu input RC/Serial...");
  Serial.println("---------------------------------");
}

// --- Variabel Global Telemetri ---
float heading = 0.0;
double lat = 0.0, lon = 0.0;
double speed = 0.0; // km/jam
int sats = 0;
// ---------------------------------

// ================= LOOP UTAMA =================
void loop() {
  // --- 1. Baca Semua Sensor ---
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Update variabel global telemetri
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  } else { lat = 0.0; lon = 0.0; } // Reset jika tidak valid

  if (gps.speed.isValid()) {
      speed = gps.speed.kmph();
  } else { speed = 0.0; }

  if (gps.satellites.isValid()) {
    sats = gps.satellites.value();
  } else { sats = 0; }

  heading = readCompass();
  if (heading == -1) { heading = 0.0; /* Handle error? */ }

  // --- 2. Baca Perintah Serial dari Jetson ---
  checkSerialInput();

  // --- 3. Baca Input RC ---
  int ch1_rudder = readChannel(0);
  int ch3_throttle = readChannel(2);
  int ch5_mode = readChannel(4); // < 1500 Manual, >= 1500 Auto
  int ch6_wp_ctrl = readChannel(5); // Kontrol Waypoint
  int ch8_aux = readChannel(7);

  // --- 4. Tentukan Mode & Eksekusi Logika ---

  // ----------- MODE MANUAL (RC Override) -----------
  if (ch5_mode < 1500) {
    if (!isManual) {
      Serial.println("? >> Mode MANUAL (RC Aktif) <<");
      // Set netral aktuator saat beralih mode
      rudderServo.write(90);
      motorESC.writeMicroseconds(1500);
      auxOutput.writeMicroseconds(1500);
      isManual = true;
      // Reset state mode rekam/simpan
      wasInCaptureMode = false;
      wasInSaveMode = false;
    }

    // Kontrol langsung dari RC
    int servoPosManual = map(ch1_rudder, 1000, 2000, 0, 180);
    rudderServo.write(servoPosManual);
    motorESC.writeMicroseconds(ch3_throttle);
    auxOutput.writeMicroseconds(ch8_aux);

    // Logika Simpan Waypoint via RC (CH6)
    if (ch6_wp_ctrl >= 1400 && ch6_wp_ctrl <= 1600) { // Posisi tengah: Siap Rekam
      if (!wasInCaptureMode) {
        Serial.println("? Mode Rekam WP Aktif (RC)");
        wasInCaptureMode = true;
        captureTriggered = false;
      }
      wasInSaveMode = false;
    } else if (ch6_wp_ctrl > 1900) { // Posisi atas: Rekam Titik
      if (wasInCaptureMode && !captureTriggered) {
        if (wasInSaveMode) { clearAllData(); wasInSaveMode = false; } // Hapus jika sebelumnya save

        if (dataIndex >= MAX_DATA) {
          Serial.println("? Memori WP Penuh!");
        } else if (gps.location.isValid()) {
          latitudes[dataIndex] = lat;
          longitudes[dataIndex] = lon;
          dataIndex++;
          saveDataToMemory(); // Simpan langsung
          Serial.print("? Titik WP "); Serial.print(dataIndex); Serial.println(" direkam.");
        } else {
          Serial.println("? GPS belum valid, WP tidak direkam.");
        }
        captureTriggered = true; // Hanya rekam sekali
      }
      // wasInCaptureMode = false; // Tetap aktif sampai CH6 netral?
    } else if (ch6_wp_ctrl < 1100) { // Posisi bawah: Simpan/Tampilkan
      if (!wasInSaveMode) {
        // saveDataToMemory(); // Sudah disimpan per titik
        Serial.println("? Menampilkan Waypoint Tersimpan...");
        displayAllData();
        wasInSaveMode = true;
      }
      wasInCaptureMode = false;
    } else { // Posisi netral: Reset state
      wasInCaptureMode = false;
      wasInSaveMode = false;
      captureTriggered = false;
    }

    // Kirim Telemetri Mode Manual (Termasuk Lat/Lon)
    Serial.print("DATA:MANUAL,");
    Serial.print(heading, 1);
    Serial.print(",");
    Serial.print(speed, 1); // kmph
    Serial.print(",");
    Serial.print(sats);
    Serial.print(",");
    Serial.print(servoPosManual);
    Serial.print(",");
    Serial.print(ch3_throttle); // Nilai PWM throttle RC
    Serial.print(",");         // Tambahan koma
    Serial.print(lat, 6);      // Tambah Latitude
    Serial.print(",");         // Tambahan koma
    Serial.println(lon, 6);    // Tambah Longitude (println di akhir)

  }
  // ----------- MODE AUTO (Kontrol Jetson atau Waypoint Internal) -----------
  else {
    if (isManual) {
      Serial.println("? >> Mode AUTO Aktif <<");
      // Set netral aktuator saat beralih mode
      rudderServo.write(90);
      motorESC.writeMicroseconds(1500);
      isManual = false;
      counter = 0; // Reset counter waypoint
    }

    // Prioritaskan Perintah 'A' (AI) dari Jetson
    if (serialCommand == 'A') {
      rudderServo.write(ai_servo_val);
      motorESC.writeMicroseconds(ai_motor_val);

      // Kirim Konfirmasi & Telemetri Mode AI (Termasuk Lat/Lon)
      Serial.print("DATA:AUTO,AI_MODE_ACTIVATED,SERVO:");
      Serial.print(ai_servo_val);
      Serial.print(",MOTOR:");
      Serial.print(ai_motor_val); // Ganti println -> print
      Serial.print(",");         // Tambah koma
      Serial.print(lat, 6);      // Tambah Latitude
      Serial.print(",");         // Tambah koma
      Serial.println(lon, 6);    // Tambah Longitude (println di akhir)

    }
    // Jika Perintah 'W' (Waypoint) dari Jetson
    else if (serialCommand == 'W') {
      // Jalankan Navigasi Waypoint Internal ESP32
      if (dataIndex > 0 && gps.location.isValid()) {
        if (counter >= dataIndex) { // Misi Selesai
          rudderServo.write(90);
          motorESC.writeMicroseconds(1500); // Berhenti (netral)
          Serial.println("DATA:AUTO,WAYPOINT_COMPLETED");
        } else { // Navigasi ke Waypoint[counter]
          double targetLat = latitudes[counter];
          double targetLon = longitudes[counter];
          double dist = haversine(lat, lon, targetLat, targetLon);
          double targetBearing = bearing(lat, lon, targetLat, targetLon);

          // PID Control
          double errorHeading = targetBearing - heading;
          if (errorHeading > 180) errorHeading -= 360;
          if (errorHeading < -180) errorHeading += 360;
          int servoPosAuto = PID_servo(targetBearing, heading);
          rudderServo.write(servoPosAuto);

          // Kontrol Kecepatan Sederhana
          int motorSpeedAuto = 1700; // Default speed
          if (dist < 3.0) motorSpeedAuto = 1600;
          if (dist < 1.0) motorSpeedAuto = 1500;
          motorESC.writeMicroseconds(motorSpeedAuto);

          // Cek Jarak & Pindah Waypoint
          if (dist < 1.5) {
            counter++;
            Serial.print("? Mencapai WP, lanjut ke #"); Serial.println(counter + 1);
          }

          // Kirim Telemetri Navigasi Waypoint (Termasuk Lat/Lon)
          Serial.print("DATA:AUTO,WAYPOINT,");
          Serial.print(counter + 1); // WP target (1-based index)
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
          Serial.print(sats);     // Ganti println -> print
          Serial.print(",");      // Tambah koma
          Serial.print(lat, 6);   // Tambah Latitude
          Serial.print(",");      // Tambah koma
          Serial.println(lon, 6); // Tambah Longitude (println di akhir)
        }
      } else { // Kondisi Error di Mode Waypoint
        rudderServo.write(90);
        motorESC.writeMicroseconds(1500); // Netral
        if (dataIndex == 0) Serial.println("DATA:AUTO,NO_WAYPOINTS");
        else Serial.println("DATA:AUTO,GPS_INVALID");
      }
    }
    // Abaikan serialCommand lain jika ada
  }

  delay(100); // Jeda utama loop
}