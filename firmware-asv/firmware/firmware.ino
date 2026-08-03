#include <Wire.h> // Library untuk komunikasi I2C (untuk CMPS12)
#include <SparkFun_Ublox_Arduino_Library.h> // Library untuk GPS U-Blox
#include <ESP32Servo.h> // Library untuk mengontrol Servo dan ESC
#include <Preferences.h> // Library untuk menyimpan data di memori non-volatile (NVS/EEPROM)
#include <ArduinoJson.h> // Library untuk memproses dan mengirim data JSON (telemetri)

// ---------------- GPS (Diganti ke U-Blox 10Hz) ----------------
SFE_UBLOX_GPS myGPS;
HardwareSerial gpsSerial(2); // Menggunakan Serial Port 2 ESP32
#define GPS_BAUD_RATE 9600 // Baud rate komunikasi ke modul GPS
#define GPS_RX_PIN 16 // Pin RX untuk komunikasi GPS (default Serial 2 RX)
#define GPS_TX_PIN 17 // Pin TX untuk komunikasi GPS (default Serial 2 TX)

// ---------------- LED GPS FIX ----------------
#define LED_GPS 2   // LED ESP32 (GPIO 2) untuk indikasi lock GPS

// ---------------- CMPS12 ----------------
#define CMPS12_ADDRESS 0x60 // Alamat I2C Kompas CMPS12
#define ANGLE_16BIT_REGISTER 2 // Register untuk membaca heading 16-bit

// ---------------- Konfigurasi Pin Aktuator ----------------
// Pin Servo Kemudi
#define PIN_SERVO_KIRI 32
#define PIN_SERVO_KANAN 23

// Pin ESC Motor
#define PIN_ESC_DEPAN_KIRI 26
#define PIN_ESC_DEPAN_KANAN 33
#define PIN_ESC_BAWAH_KIRI 27
#define PIN_ESC_BAWAH_KANAN 25

// Pin Arah (Maju/Mundur) menggunakan sinyal PWM
#define DIR_DEPAN_KIRI 14
#define DIR_DEPAN_KANAN 19
#define DIR_BAWAH_KIRI 15
#define DIR_BAWAH_KANAN 18

Servo servoKiri; 
Servo servoKanan; 

Servo escDepanKiri; 
Servo escDepanKanan; 
Servo escBawahKiri; 
Servo escBawahKanan; 

Servo dirDepanKiri;
Servo dirDepanKanan;
Servo dirBawahKiri;
Servo dirBawahKanan;

// ---------------- PID ----------------
double Kp = 2.0, Ki = 0.0, Kd = 0.5; // Konstanta PID

// --- Waypoint Inversion Control for AI Mode ---
// Fitur inversi servo dihapus

double error, lastError = 0, integral = 0;

// ---------------- LOS Navigation ----------------
double L_delta = 3.0;
double prev_wp_lat = 0.0;
double prev_wp_lon = 0.0;
bool is_new_wp = true;
double cross_track_error = 0.0;

// ---------------- Waypoint ----------------
#define MAX_DATA 20 
Preferences preferences; 

float latitudes[MAX_DATA]; 
float longitudes[MAX_DATA]; 
int dataIndex = 0; 
int counter = 0; 
int targetCaptureIndex = -1; // Target index untuk Replace WP via RC

bool captureTriggered = false; 
bool wasInCaptureMode = false; 
bool wasInSaveMode = false; 

// --- KONTROL DARI JETSON/KOMUNIKASI SERIAL ---
char serialCommand = 'W'; 
int ai_servo_val = 90; 
int ai_motor_val = 1500; // Untuk motor bawah
int ai_motor_depan_kiri_val = 1000;  // Nilai dari serial Jetson untuk motor depan kiri
int ai_motor_depan_kanan_val = 1000; // Nilai dari serial Jetson untuk motor depan kanan

// --- Buffer JSON & Serial ---
StaticJsonDocument<400> jsonDoc;
String serialInputBuffer = ""; 

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
      Serial.print(i);
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

// --- FUNGSI MEMBACA PERINTAH SERIAL ---
void checkSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read(); 
    
    if (incomingChar == '\n') {
      serialInputBuffer.trim(); 
      
      if (serialInputBuffer.length() > 0) {
        serialCommand = serialInputBuffer.charAt(0); 
        
        if (serialCommand == 'A') {
          // FORMAT BARU: A,<servo>,<motor_bawah>,<motor_depan_kiri>,<motor_depan_kanan>
          int comma1 = serialInputBuffer.indexOf(',');
          int comma2 = serialInputBuffer.indexOf(',', comma1 + 1);
          int comma3 = serialInputBuffer.indexOf(',', comma2 + 1);
          int comma4 = serialInputBuffer.indexOf(',', comma3 + 1);

          // Cek apakah ada minimal 4 koma sebelum nilai diekstrak
          if (comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0) {
            String servoStr          = serialInputBuffer.substring(comma1 + 1, comma2);
            String motorBwhStr       = serialInputBuffer.substring(comma2 + 1, comma3);
            String motorDepanKiriStr = serialInputBuffer.substring(comma3 + 1, comma4);
            String motorDepanKananStr= serialInputBuffer.substring(comma4 + 1);

            ai_servo_val             = servoStr.toInt();
            ai_motor_val             = motorBwhStr.toInt();
            ai_motor_depan_kiri_val  = motorDepanKiriStr.toInt();
            ai_motor_depan_kanan_val = motorDepanKananStr.toInt();
          }
        } else if (serialCommand == 'C') {
          int comma = serialInputBuffer.indexOf(',');
          if (comma > 0) {
            String cmdAction = serialInputBuffer.substring(comma + 1);
            if (cmdAction == "INC") {
              counter++;
              if (counter > dataIndex) counter = dataIndex;
            } else if (cmdAction == "DEC") {
              if (counter > 0) counter--;
            } else if (cmdAction == "RESET") {
              counter = 0;
            }
          }
        } else if (serialCommand == 'P') {
          int comma1 = serialInputBuffer.indexOf(',');
          if (comma1 > 0) {
            String subCmd = serialInputBuffer.substring(comma1 + 1);
            if (subCmd == "CLEAR") {
              clearAllData();
              counter = 0;
            } else if (subCmd == "SAVE") {
              saveDataToMemory();
              displayAllData();
            } else if (subCmd.startsWith("ADD,")) {
              int comma2 = subCmd.indexOf(',');
              int comma3 = subCmd.indexOf(',', comma2 + 1);
              if (comma2 > 0 && comma3 > 0) {
                String latStr = subCmd.substring(comma2 + 1, comma3);
                String lonStr = subCmd.substring(comma3 + 1);
                if (dataIndex < MAX_DATA) {
                  latitudes[dataIndex] = latStr.toFloat();
                  longitudes[dataIndex] = lonStr.toFloat();
                  dataIndex++;
                }
              }
            } else if (subCmd.startsWith("ARM,")) {
              int comma2 = subCmd.indexOf(',');
              if (comma2 > 0) {
                String idxStr = subCmd.substring(comma2 + 1);
                targetCaptureIndex = idxStr.toInt();
                Serial.println("Target bidikan RC disetel ke: " + String(targetCaptureIndex));
              }
            }
          }
        }
      }
      serialInputBuffer = "";
    } else {
      if (serialInputBuffer.length() < 128) { 
        serialInputBuffer += incomingChar;
      }
    }
  }
}

void setup() {
  Serial.begin(230400); 
  serialInputBuffer.reserve(128);

  // ========================================================
  // 1. INISIALISASI AKTUATOR TERLEBIH DAHULU (ESC & MAJU/MUNDUR)
  // ========================================================
  Serial.println("Menginisialisasi ESC, Pin Arah, dan Servo...");
  
  // --- Inisialisasi ESC Motor ---
  escDepanKiri.attach(PIN_ESC_DEPAN_KIRI);
  escDepanKanan.attach(PIN_ESC_DEPAN_KANAN);
  escBawahKiri.attach(PIN_ESC_BAWAH_KIRI);
  escBawahKanan.attach(PIN_ESC_BAWAH_KANAN);

  // --- Inisialisasi Pin Arah (Sebagai Servo/PWM) ---
  dirDepanKiri.attach(DIR_DEPAN_KIRI);
  dirDepanKanan.attach(DIR_DEPAN_KANAN);
  dirBawahKiri.attach(DIR_BAWAH_KIRI);
  dirBawahKanan.attach(DIR_BAWAH_KANAN);

  // --- Inisialisasi Servo Kemudi ---
  servoKiri.attach(PIN_SERVO_KIRI);
  servoKanan.attach(PIN_SERVO_KANAN);

  // ========================================================
  // 2. INISIALISASI SENSOR & SISTEM LAINNYA
  // ========================================================
  // --- LED GPS FIX ---
  pinMode(LED_GPS, OUTPUT);
  digitalWrite(LED_GPS, LOW); 

  Serial.println("Mencoba koneksi ke GPS U-Blox...");
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 

  if (myGPS.begin(gpsSerial)) {
    Serial.println("Koneksi GPS berhasil!");
  } else {
    Serial.println("Gagal koneksi ke GPS. Cek kabel & baud rate.");
  }

  myGPS.setUART1Output(COM_TYPE_UBX); 
  myGPS.setNavigationFrequency(10); 
  myGPS.setAutoPVT(true); 

  Wire.begin(21, 22); 

  // Inisialisasi PPM
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
double speed = 0.0; 
int sats = 0;
// ---------------------------------

void loop() {

  // --- 1. Baca Sensor GPS ---
  if (myGPS.getPVT()) { 
    uint8_t fixType = myGPS.getFixType(); 

    if (fixType > 0) digitalWrite(LED_GPS, HIGH);
    else digitalWrite(LED_GPS, LOW);

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

  // Baca Channel Radio (PPM)
  int ch5 = readChannel(4); 
  int ch6 = readChannel(5); 

  String mode = "MANUAL";
  String status = "ACTIVE";
  
  // Variabel penampung output aktuator di cycle ini
  int finalServo = 90;
  int finalMotor = 1500;           
  int finalMotorDepanKiri = 1000;  // Pastikan default 1000 (mati)
  int finalMotorDepanKanan = 1000; // Pastikan default 1000 (mati)
  int finalDir = 1500;             
  
  int wp_target_idx = 0;
  double wp_dist_m = 0.0;
  double wp_target_brg = 0.0;
  double wp_error_hdg = 0.0;

  // ----------------- MANUAL MODE -----------------
  if (ch5 < 1500) { 
    if (!isManual) {
      Serial.println("Switching to MANUAL...");
      isManual = true;
      wasInCaptureMode = false;
      wasInSaveMode = false;
    }

    mode = "MANUAL";

    int ch1 = readChannel(0); 
    finalServo = map(ch1, 1000, 2000, 0, 180); 

    int ch3 = readChannel(2); 
    finalMotor = ch3;

    finalMotorDepanKiri = 1000;
    finalMotorDepanKanan = 1000;

    int ch8 = readChannel(7);
    finalDir = ch8;

    if (ch6 > 1900) { 
      // Posisi 3 (Bawah) - Siap-siap merekam
      if (!wasInCaptureMode) {
        Serial.println("🟡 MODE REKAM: Siap merekam waypoint baru.");
        wasInCaptureMode = true;
        captureTriggered = false;
      }
    } else if (ch6 >= 1400 && ch6 <= 1600) { 
      // Posisi 2 (Tengah) - Merekam (Capture) 1 titik
      if (wasInCaptureMode && !captureTriggered) {
        if (wasInSaveMode) {
          if (targetCaptureIndex == -1) {
            clearAllData();
            Serial.println("♻ Memulai sesi perekaman baru (Data lama dihapus).");
          } else {
            Serial.println("✏ Mode Replace aktif: Tidak menghapus data lama.");
          }
          wasInSaveMode = false;
        }
        if (dataIndex >= MAX_DATA) {
          Serial.println("⚠ Memori penuh. Tidak bisa menambah titik lagi.");
        } else {
          if (myGPS.getFixType() > 0) { 
            if (targetCaptureIndex >= 0 && targetCaptureIndex < dataIndex) {
              latitudes[targetCaptureIndex] = lat;
              longitudes[targetCaptureIndex] = lon;
              saveDataToMemory();
              Serial.println("📍 Titik ke-" + String(targetCaptureIndex) + " diganti (Replace).");
              targetCaptureIndex = -1; // Reset target
            } else {
              latitudes[dataIndex] = lat;
              longitudes[dataIndex] = lon;
              dataIndex++;
              saveDataToMemory();
              Serial.println("📍 Titik ke-" + String(dataIndex) + " direkam.");
            }
          } else {
            Serial.println("❌ GPS belum lock. Tidak dapat menambah data.");
          }
        }
        captureTriggered = true;
      }
      wasInCaptureMode = false;
    } else if (ch6 < 1100) { 
      // Posisi 1 (Atas) - Save semua & Sync ke GUI
      if (!wasInSaveMode) {
        saveDataToMemory();
        Serial.println("✅ Semua waypoint tersimpan.");
        displayAllData();
        
        // [SYNC KE JETSON] Mengirimkan semua waypoint ke Jetson untuk GUI
        Serial.println("SYNC_WP_START");
        for (int i = 0; i < dataIndex; i++) {
          Serial.print("SYNC_WP,");
          Serial.print(i);
          Serial.print(",");
          Serial.print(latitudes[i], 6);
          Serial.print(",");
          Serial.println(longitudes[i], 6);
        }
        Serial.println("SYNC_WP_END");
        
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
      is_new_wp = true;
    }

    mode = "AUTO";
    finalDir = 1500; 

    if (serialCommand == 'A') {
      int calculatedServoVal = ai_servo_val;
      
      // Pada mode AUTO, kontrol motor sepenuhnya dari Jetson via Serial
      // Inversi servo otomatis berdasarkan waypoint index DITIADAKAN
      finalServo = ai_servo_val;
      finalMotor = ai_motor_val;
      finalDir = 1500;
      finalMotorDepanKiri = ai_motor_depan_kiri_val;
      finalMotorDepanKanan = ai_motor_depan_kanan_val;
      status = "AI_ACTIVE";
      
    } 
    else if (serialCommand == 'W') {
      status = "WAYPOINT";
      
      if (dataIndex > 0 && myGPS.getFixType() > 0) { 
        
        if (counter >= dataIndex) { 
          finalServo = 90;
          finalMotor = 1000; 
          finalMotorDepanKiri = 1000;  // Paksa mati di Mode W
          finalMotorDepanKanan = 1000; // Paksa mati di Mode W
          status = "WP_COMPLETE";
          wp_target_idx = dataIndex; 
        } else { 
          double targetLat = latitudes[counter];
          double targetLon = longitudes[counter];
          
          if (is_new_wp) {
            if (counter == 0) {
              prev_wp_lat = lat;
              prev_wp_lon = lon;
            } else {
              prev_wp_lat = latitudes[counter - 1];
              prev_wp_lon = longitudes[counter - 1];
            }
            is_new_wp = false;
          }

          double dist = haversine(lat, lon, targetLat, targetLon); 
          
          double path_angle = bearing(prev_wp_lat, prev_wp_lon, targetLat, targetLon);
          double dist_from_prev = haversine(prev_wp_lat, prev_wp_lon, lat, lon);
          double bearing_from_prev = bearing(prev_wp_lat, prev_wp_lon, lat, lon);
          
          cross_track_error = dist_from_prev * sin(radians(bearing_from_prev - path_angle));
          
          double los_correction = degrees(atan2(-cross_track_error, L_delta));
          double targetBearing = fmod((path_angle + los_correction + 360.0), 360.0);
          
          double errorHeading = targetBearing - heading;
          if (errorHeading > 180) errorHeading -= 360;
          if (errorHeading < -180) errorHeading += 360;
          
          int servoPos = PID_servo(targetBearing, heading);
          finalServo = servoPos;

          int motorSpeed = readChannel(6);  
          finalMotor = motorSpeed;          
          
          finalMotorDepanKiri = 1000;       // Paksa mati di Mode W
          finalMotorDepanKanan = 1000;      // Paksa mati di Mode W

          if (dist < 1.75) {
            counter++; 
            is_new_wp = true;
            Serial.print("✅ WP #");
            Serial.print(counter);
            Serial.println(" tercapai. Menuju WP berikutnya.");
          }

          wp_target_idx = counter; 
          wp_dist_m = dist;
          wp_target_brg = targetBearing;
          wp_error_hdg = errorHeading;
        }
        
      } else {
        finalServo = 90;
        finalMotor = 1000; 
        finalMotorDepanKiri = 1000;   // Paksa mati di Mode W
        finalMotorDepanKanan = 1000;  // Paksa mati di Mode W
        if (dataIndex == 0) status = "NO_WAYPOINTS";
        else status = "GPS_INVALID";
      }
    }
  }

  // ========================================
  // --- 3. Kontrol Aktuator Lanjutan ---
  // ========================================

  servoKiri.write(finalServo); 
  servoKanan.write(finalServo); 

  dirDepanKiri.writeMicroseconds(finalDir);
  dirDepanKanan.writeMicroseconds(finalDir);
  dirBawahKiri.writeMicroseconds(finalDir);
  dirBawahKanan.writeMicroseconds(finalDir);

  escDepanKiri.writeMicroseconds(finalMotorDepanKiri);
  escDepanKanan.writeMicroseconds(finalMotorDepanKanan);
  
  escBawahKiri.writeMicroseconds(finalMotor);
  escBawahKanan.writeMicroseconds(finalMotor);

  // ========================================
  // --- 4. BLOK TELEMETRI JSON ---
  // ========================================
  
  jsonDoc.clear(); 

  jsonDoc["mod"] = mode;
  jsonDoc["sts"] = status;

  jsonDoc["hdg"] = (float)round(heading * 100) / 100;
  jsonDoc["lat"] = lat;
  jsonDoc["lon"] = lon;
  jsonDoc["spd"] = (float)round(speed * 100) / 100;
  jsonDoc["sat"] = sats;

  jsonDoc["srv"] = finalServo;
  jsonDoc["mot"] = finalMotor;
  jsonDoc["m_dl"] = finalMotorDepanKiri;
  jsonDoc["m_dr"] = finalMotorDepanKanan;
  
  if (serialCommand == 'A') {
    // Mode Auto: Data inversi telah dihapus untuk menghemat bandwidth
  }

  jsonDoc["w_id"] = counter;
  jsonDoc["w_tot"] = dataIndex;

  if (mode == "AUTO" && serialCommand == 'W') { 
    if (status == "WP_COMPLETE") {
      jsonDoc["w_dst"] = 0.0;
      jsonDoc["w_brg"] = 0.0;
      jsonDoc["w_err"] = 0.0;
      jsonDoc["xte"] = 0.0;
    } else {
      jsonDoc["w_dst"] = (float)round(wp_dist_m * 100) / 100;
      jsonDoc["w_brg"] = (float)round(wp_target_brg * 100) / 100;
      jsonDoc["w_err"] = (float)round(wp_error_hdg * 100) / 100;
      jsonDoc["xte"] = (float)round(cross_track_error * 100) / 100;
    }
  }

  serializeJson(jsonDoc, Serial);
  Serial.println(); 

  delay(80); 
}