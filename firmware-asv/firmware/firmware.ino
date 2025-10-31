#include <Wire.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <ArduinoJson.h> // <-- [BARU] DITAMBAHKAN UNTUK JSON

// ---------------- GPS (Diganti ke U-Blox 10Hz) ----------------
SFE_UBLOX_GPS myGPS;
HardwareSerial gpsSerial(2);
#define GPS_BAUD_RATE 9600 // Sesuaikan dengan setting U-Blox Anda (9600 atau 115200)
#define GPS_RX_PIN 16 // Pin default Serial 2 (RX)
#define GPS_TX_PIN 17 // Pin default Serial 2 (TX)

// ---------------- CMPS12 ----------------
#define CMPS12_ADDRESS 0x60
#define ANGLE_16BIT_REGISTER 2

// ---------------- Servo dan ESC ----------------
Servo rudderServo;
Servo motorESC;
Servo auxOutput;

// ---------------- PID ----------------
double Kp = 2.0, Ki = 0.0, Kd = 0.5;
double error, lastError = 0, integral = 0;

// ---------------- Waypoint ----------------
#define MAX_DATA 15
Preferences preferences;

float latitudes[MAX_DATA];
float longitudes[MAX_DATA];
int dataIndex = 0;
int counter = 0;

bool captureTriggered = false;
bool wasInCaptureMode = false;
bool wasInSaveMode = false;

// --- [FIX] VARIABEL BARU UNTUK KONTROL DARI JETSON ---
char serialCommand = 'W';
int ai_servo_val = 90;
int ai_motor_val = 1500;

// --- [BARU] Buffer JSON Global ---
StaticJsonDocument<300> jsonDoc;

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
  error = input - setpoint;

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
#define CHANNELS 10
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
  Serial.println("🗑 Semua data lama telah dihapus.");
}

void displayAllData() {
  if (dataIndex > 0) {
    Serial.println("📋 DATA KOORDINAT TERSIMPAN:");
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
    Serial.println("📋 Tidak ada data koordinat yang tersimpan.");
  }
}

// ---------------- MODE FLAG ----------------
bool isManual = true;

// --- [FIX] FUNGSI BARU UNTUK MEMBACA PERINTAH SERIAL ---
void checkSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      serialCommand = input.charAt(0);
    }

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
  }
}


void setup() {
  Serial.begin(115200);
  
  // --- Inisialisasi GPS U-Blox (Baru) ---
  Serial.println("Mencoba koneksi ke GPS U-Blox...");
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (myGPS.begin(gpsSerial)) {
    Serial.println("Koneksi GPS berhasil!");
  } else {
    Serial.println("Gagal koneksi ke GPS. Cek kabel & baud rate.");
  }

  // --- Konfigurasi GPS untuk UBX 10Hz ---
  myGPS.setUART1Output(COM_TYPE_UBX);
  myGPS.setNavigationFrequency(10);
  myGPS.setAutoPVT(true);

  uint8_t navFreq = myGPS.getNavigationFrequency();
  if (navFreq == 10) {
    Serial.println("GPS berhasil dikonfigurasi ke 10Hz.");
  } else {
    Serial.print("Konfigurasi 10Hz GAGAL! Frekuensi saat ini: ");
    Serial.print(navFreq);
    Serial.println(" Hz");
  }

  Wire.begin(21, 22);

  rudderServo.attach(18);
  motorESC.attach(19);
  auxOutput.attach(23);

  rudderServo.write(90);
  motorESC.writeMicroseconds(1500);
  auxOutput.writeMicroseconds(1500);

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  loadDataFromMemory();

  Serial.println("");
  Serial.println("🌍 GPS + WAYPOINT SYSTEM (INTEGRATED)");
  Serial.println("================================");
  Serial.print("Data tersimpan: ");
  Serial.print(dataIndex);
  Serial.print("/");
  Serial.print(MAX_DATA);
  Serial.println(" titik");
  Serial.println("================================");
}

// --- Variabel Global Telemetri ---
float heading = 0.0;
double lat = 0.0, lon = 0.0;
double speed = 0.0; // km/jam
int sats = 0;
// ---------------------------------

void loop() {
  // --- 1. Baca Sensor ---
  if (myGPS.getPVT()) {
    uint8_t fixType = myGPS.getFixType();
    if (fixType > 0) {
        lat = myGPS.getLatitude() / 10000000.0;
        lon = myGPS.getLongitude() / 10000000.0;
        speed = myGPS.getGroundSpeed() / 1000.0 * 3.6;
        sats = myGPS.getSIV();
    } else {
        lat = 0.0;
        lon = 0.0;
        speed = 0.0;
        sats = 0;
    }
  }
  
  checkSerialInput();
  
  heading = readCompass();
  if (heading == -1) { heading = 0.0; }

  int ch5 = readChannel(4);
  int ch6 = readChannel(5);

  // --- Variabel Penampung Status ---
  String mode = "MANUAL";
  String status = "ACTIVE";
  int finalServo = 90;
  int finalMotor = 1500;
  
  int wp_target_idx = 0;
  double wp_dist_m = 0.0;
  double wp_target_brg = 0.0;
  double wp_error_hdg = 0.0;


  // ----------------- MANUAL MODE -----------------
  if (ch5 < 1500) {
    if (!isManual) {
      Serial.println("Switching to MANUAL...");
      auxOutput.writeMicroseconds(1500); 
      isManual = true;
      wasInCaptureMode = false;
      wasInSaveMode = false;
    }

    mode = "MANUAL";

    int ch1 = readChannel(0);
    int servoPosManual = map(ch1, 1000, 2000, 0, 180);
    finalServo = servoPosManual;

    int ch3 = readChannel(2);
    finalMotor = ch3;

    int ch8 = readChannel(7);
    auxOutput.writeMicroseconds(ch8);
    
    // --- Logika Perekaman Waypoint (Tetap Sama) ---
    if (ch6 >= 1400 && ch6 <= 1600) {
      if (!wasInCaptureMode) {
        Serial.println("🟡 MODE REKAM: Siap merekam waypoint baru.");
        wasInCaptureMode = true;
        captureTriggered = false;
      }
    } else if (ch6 > 1900) {
      if (wasInCaptureMode && !captureTriggered) {
        if (wasInSaveMode) {
          clearAllData();
          wasInSaveMode = false;
        }
        if (dataIndex >= MAX_DATA) {
          Serial.println("⚠ Memori penuh. Tidak bisa menambah titik lagi.");
        } else {
          if (myGPS.getFixType() > 0) {
            latitudes[dataIndex] = lat;
            longitudes[dataIndex] = lon;
            dataIndex++;
            saveDataToMemory();
            Serial.println("📍 Titik ke-" + String(dataIndex) + " direkam.");
          } else {
            Serial.println("❌ GPS belum lock. Tidak dapat menambah data.");
          }
        }
        captureTriggered = true;
      }
      wasInCaptureMode = false;
    } else if (ch6 < 1100) {
      if (!wasInSaveMode) {
        saveDataToMemory();
        Serial.println("✅ Semua waypoint tersimpan.");
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
      isManual = false;
      counter = 0;
    }

    mode = "AUTO";

    if (serialCommand == 'A') {
      // PRIORITAS 1: Perintah AI
      finalServo = ai_servo_val;
      finalMotor = ai_motor_val;
      status = "AI_ACTIVE";
    } 
    else if (serialCommand == 'W') {
      // PRIORITAS 2: Waypoint
      status = "WAYPOINT";
      
      if (dataIndex > 0 && myGPS.getFixType() > 0) {
        if (counter >= dataIndex) {
          // Selesai
          finalServo = 90;
          finalMotor = 1000;
          status = "WP_COMPLETE";
        } else {
          // Navigasi
          double targetLat = latitudes[counter];
          double targetLon = longitudes[counter];
          double dist = haversine(lat, lon, targetLat, targetLon);
          double targetBearing = bearing(lat, lon, targetLat, targetLon);
          double errorHeading = targetBearing - heading;
          if (errorHeading > 180) errorHeading -= 360;
          if (errorHeading < -180) errorHeading += 360;
          
          int servoPos = PID_servo(targetBearing, heading);
          finalServo = servoPos;
  
          int motorSpeed = 1700; 
          if (dist < 3.0) motorSpeed = 1600;
          if (dist < 1.0) motorSpeed = 1500;
          finalMotor = motorSpeed;
          
          if (dist < 1.75) {
            counter++;
          }
  
          // Simpan data WP untuk JSON
          wp_target_idx = counter + 1;
          wp_dist_m = dist;
          wp_target_brg = targetBearing;
          wp_error_hdg = errorHeading;
        }
      } else {
        // Error
        finalServo = 90;
        finalMotor = 1500;
        if (dataIndex == 0) {
          status = "NO_WAYPOINTS";
        } else {
          status = "GPS_INVALID";
        }
      }
    }
  }

  // --- 3. Kontrol Aktuator (Satu Titik) ---
  rudderServo.write(finalServo);
  motorESC.writeMicroseconds(finalMotor);

  
  // ========================================
  // --- 4. BLOK TELEMETRI JSON BARU ---
  // ========================================
  
  jsonDoc.clear();

  jsonDoc["mode"] = mode;
  jsonDoc["status"] = status;

  // Data Sensor Inti (SELALU DIKIRIM)
  jsonDoc["heading"] = (float)round(heading * 100) / 100;
  jsonDoc["lat"] = lat;
  jsonDoc["lon"] = lon;
  jsonDoc["speed_kmh"] = (float)round(speed * 100) / 100;
  jsonDoc["sats"] = sats;

  // Data Output Aktuator (SELALU DIKIRIM)
  jsonDoc["servo_out"] = finalServo;
  jsonDoc["motor_out"] = finalMotor;

  // Data khusus Waypoint (HANYA DIKIRIM JIKA PERLU)
  if (mode == "AUTO" && status == "WAYPOINT") {
    jsonDoc["wp_target_idx"] = wp_target_idx;
    jsonDoc["wp_dist_m"] = (float)round(wp_dist_m * 100) / 100;
    jsonDoc["wp_target_brg"] = (float)round(wp_target_brg * 100) / 100;
    jsonDoc["wp_error_hdg"] = (float)round(wp_error_hdg * 100) / 100;
  }

  // Kirim JSON ke Serial
  serializeJson(jsonDoc, Serial);
  Serial.println(); // PENTING: Kirim newline sebagai penanda akhir baris
  
  delay(50);
}