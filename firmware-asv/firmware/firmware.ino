#include <Wire.h>           // Library untuk komunikasi I2C (untuk CMPS12 & OLED)
#include <ESP32Servo.h>     // Library untuk mengontrol Servo dan ESC
#include <Preferences.h>    // Library untuk menyimpan data di memori non-volatile (NVS/EEPROM)
#include <ArduinoJson.h>    // Library untuk memproses dan mengirim data JSON (telemetri)
#include <Adafruit_GFX.h>   // Library grafis dasar untuk OLED
#include <Adafruit_SSD1306.h> // Library driver OLED SSD1306 (DSP-0109)

// ---------------- OLED DSP-0109 (SSD1306, 128x64) ----------------
#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_ADDR     0x3C // Alamat I2C default SSD1306
#define OLED_RESET    -1   // Reset pin tidak digunakan (berbagi dengan ESP32 RST)
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
bool oledOk = false;             // Flag: OLED berhasil diinisialisasi
unsigned long lastOledUpdate = 0; // Timestamp terakhir OLED diperbarui

// ======================================================
// ESP32 GPS 5 Hz RECEIVER (ZED-F9P NMEA @ 115200 Baud)
// Tanpa library GPS - Parsing NMEA GGA & RMC
// ======================================================
#define GPS_RX 16
#define GPS_TX 17
#define GPS_SERIAL Serial2
#define GPS_BAUD 115200

// --- DATA GPS GLOBAL ---
double latitude  = 0.0;
double longitude = 0.0;
double altitude  = 0.0;
double hdop      = 0.0;
double speedKmh  = 0.0;
int satellites   = 0;
String fixStatus = "NO FIX";
uint8_t gpsFixType = 0; // 0=No fix, 1=GPS, 2=DGPS, 4=RTK FIX, 5=RTK FLOAT, 6=DR

// Aliases untuk kompatibilitas penuh logika navigasi firmware
#define lat latitude
#define lon longitude
#define speed speedKmh
#define sats satellites

// --- BUFFER & MONITORING NMEA ---
String nmeaBuffer = "";
unsigned long lastGGA = 0;
double gpsHz = 0.0;

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


double error = 0, lastError = 0, integral = 0;
bool pidInitialized = false;

void resetPIDController() {
  error = 0.0;
  lastError = 0.0;
  integral = 0.0;
  pidInitialized = false;
}

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

// --- PORTRAIT MISSION STATE MACHINE ---
enum PortraitState { PT_NORMAL, PT_SLOW, PT_STOP, PT_REVERSE };
PortraitState portraitState = PT_NORMAL;
unsigned long portraitTimer = 0;

// Konfigurasi Portrait (Default, bisa diubah dari GUI via Jetson)
int portraitSpeed = 1400;           // PWM motor utama saat pelan di segmen portrait
int portraitReverseSpeed = 1400;    // PWM motor utama saat mundur
unsigned long portraitStopMs = 3000;    // Durasi berhenti di titik akhir (ms)
unsigned long portraitReverseMs = 2000; // Durasi mundur setelah berhenti (ms)

// Range WP Portrait (Default, bisa diubah dari GUI via Jetson)
int uwStart = 11, uwEnd = 12;      // Kotak Biru (Underwater)
int surfStart = 13, surfEnd = 14;   // Kotak Hijau (Surface)

// --- DOCKING MISSION STATE MACHINE ---
enum DockingState { DK_IDLE, DK_SWING, DK_COMPLETE };
DockingState dockingState = DK_IDLE;
unsigned long dockingTimer = 0;
// Konfigurasi Docking (Default, bisa diubah dari GUI via Jetson)
int dockMotorUtama = 1200;          // PWM motor utama saat docking
unsigned long dockChargeMs = 3000;  // Durasi maju menabrak dock (ms)
int dockTurnDirection = 0;          // 0=KIRI (Arena A), 1=KANAN (Arena B)
int dockServoLeft = 180;            // Sudut servo saat docking Arena A (Patah KIRI secara fisik)
int dockServoRight = 0;             // Sudut servo saat docking Arena B (Patah KANAN secara fisik)
bool dockingEnabled = true;         // [ON/OFF] Flag aktif/nonaktif docking mission

// --- KONTROL DARI JETSON/KOMUNIKASI SERIAL ---
char serialCommand = 'W'; 
int ai_servo_val = 90; 
int ai_motor_val = 1500; // Untuk motor bawah
int ai_motor_depan_kiri_val = 1000;  // Nilai dari serial Jetson untuk motor depan kiri
int ai_motor_depan_kanan_val = 1000; // Nilai dari serial Jetson untuk motor depan kanan
int ai_dir_val = 1000;               // Arah motor AI: 1000 = Maju, 2000 = Mundur
int ai_dir_depan_kiri_val = 1000;    // Arah motor AI depan kiri: 1000 = Maju, 2000 = Mundur
int ai_dir_depan_kanan_val = 1000;   // Arah motor AI depan kanan: 1000 = Maju, 2000 = Mundur

// --- Buffer JSON & Serial ---
StaticJsonDocument<400> jsonDoc;
String serialInputBuffer = ""; 

// ---------------- Haversine ----------------
double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
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

// ---------------- CMPS12 24-POINT CALIBRATION ----------------
#define CMPS12_CAL_POINTS 24

// Data kalibrasi hasil pengukuran di posisi CMPS12 terpasang pada kapal.
// Kolom pertama = RAW CMPS12, kolom kedua = heading referensi sebenarnya.
// Urutan RAW WAJIB naik dan sudah diurutkan melewati 0/360.
const float cmpsRawCal[CMPS12_CAL_POINTS] = {
  3.5, 23.0, 31.8, 44.8, 51.3, 61.1,
  71.5, 81.7, 94.3, 104.3, 115.3, 128.7,
  141.7, 155.8, 171.4, 188.4, 209.5, 233.3,
  258.0, 282.5, 301.5, 318.7, 333.6, 348.2
};

const float cmpsRefCal[CMPS12_CAL_POINTS] = {
  345.0, 0.0, 15.0, 30.0, 45.0, 60.0,
  75.0, 90.0, 105.0, 120.0, 135.0, 150.0,
  165.0, 180.0, 195.0, 210.0, 225.0, 240.0,
  255.0, 270.0, 285.0, 300.0, 315.0, 330.0
};

float normalizeHeading360(float angle) {
  while (angle < 0.0) angle += 360.0;
  while (angle >= 360.0) angle -= 360.0;
  return angle;
}

// Selisih sudut terpendek (-180 ... +180).
float circularDifference(float target, float value) {
  float d = target - value;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}

// Koreksi RAW CMPS12 menggunakan 24 titik kalibrasi.
// Metode: interpolasi ERROR (reference - raw), bukan langsung raw -> reference.
// Cara ini membuat transisi 348 -> 360 -> 0 -> 23 tetap kontinu.
float calibrateCMPS12(float raw) {
  raw = normalizeHeading360(raw);

  // Hitung error kalibrasi setiap titik.
  float correction[CMPS12_CAL_POINTS];
  for (int i = 0; i < CMPS12_CAL_POINTS; i++) {
    correction[i] = circularDifference(cmpsRefCal[i], cmpsRawCal[i]);
  }

  // --------------------------------------------------
  // RAW 0 ... 3.5
  // Segmen melintasi 360:
  // 348.2 -> 3.5  = 330 -> 345
  // --------------------------------------------------
  if (raw < cmpsRawCal[0]) {
    float x1 = cmpsRawCal[CMPS12_CAL_POINTS - 1];
    float x2 = cmpsRawCal[0] + 360.0;
    float c1 = correction[CMPS12_CAL_POINTS - 1];
    float c2 = correction[0];

    float rawExtended = raw + 360.0;
    float t = (rawExtended - x1) / (x2 - x1);
    float c = c1 + t * (c2 - c1);

    return normalizeHeading360(raw + c);
  }

  // --------------------------------------------------
  // RAW 3.5 ... 23.0
  // 3.5 -> 23.0 = 345 -> 360/0
  // --------------------------------------------------
  if (raw < cmpsRawCal[1]) {
    float x1 = cmpsRawCal[0];
    float x2 = cmpsRawCal[1];
    float c1 = correction[0];
    float c2 = correction[1];

    float t = (raw - x1) / (x2 - x1);
    float c = c1 + t * (c2 - c1);

    return normalizeHeading360(raw + c);
  }

  // --------------------------------------------------
  // SEGMENT NORMAL
  // --------------------------------------------------
  for (int i = 1; i < CMPS12_CAL_POINTS - 1; i++) {
    if (raw >= cmpsRawCal[i] && raw < cmpsRawCal[i + 1]) {
      float x1 = cmpsRawCal[i];
      float x2 = cmpsRawCal[i + 1];
      float c1 = correction[i];
      float c2 = correction[i + 1];

      float t = (raw - x1) / (x2 - x1);
      float c = c1 + t * (c2 - c1);

      return normalizeHeading360(raw + c);
    }
  }

  // --------------------------------------------------
  // RAW 348.2 ... 360
  // 348.2 -> 3.5/360 = 330 -> 345
  // --------------------------------------------------
  if (raw >= cmpsRawCal[CMPS12_CAL_POINTS - 1]) {
    float x1 = cmpsRawCal[CMPS12_CAL_POINTS - 1];
    float x2 = cmpsRawCal[0] + 360.0;
    float c1 = correction[CMPS12_CAL_POINTS - 1];
    float c2 = correction[0];

    float t = (raw - x1) / (x2 - x1);
    float c = c1 + t * (c2 - c1);

    return normalizeHeading360(raw + c);
  }

  return raw;
}

// ---------------- Baca heading CMPS12 ----------------
// ---------------- Baca heading CMPS12 ----------------
// Tidak lagi mengembalikan heading lama saat I2C gagal.
// true = valid, false = gagal.
bool readCompass(float &outHeading) {
  Wire.beginTransmission(CMPS12_ADDRESS);
  Wire.write(ANGLE_16BIT_REGISTER);
  uint8_t i2cError = Wire.endTransmission();

  if (i2cError != 0) return false;

  uint8_t bytesReceived = Wire.requestFrom(
    (uint8_t)CMPS12_ADDRESS,
    (uint8_t)2,
    (uint8_t)1
  );

  if (bytesReceived != 2 || Wire.available() < 2) return false;

  byte highByte = Wire.read();
  byte lowByte = Wire.read();
  unsigned int angle16 = (highByte << 8) | lowByte;
  float rawHeading = angle16 / 10.0;

  if (!isfinite(rawHeading) || rawHeading < 0.0 || rawHeading >= 360.0) return false;

  float calibratedHeading = calibrateCMPS12(rawHeading);
  if (!isfinite(calibratedHeading)) return false;

  outHeading = normalizeHeading360(calibratedHeading);
  return true;
}

// ---------------- PID untuk servo ----------------
int PID_servo(double setpoint, double input) {
  error = input - setpoint;

  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  integral += error;

  double derivative = 0.0;
  if (pidInitialized) {
    derivative = error - lastError;
  } else {
    // Setelah reset, hindari derivative kick pada sampel pertama.
    pidInitialized = true;
  }
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
volatile unsigned long lastPpmFrameMicros = 0;
volatile byte lastPpmChannelCount = 0;

#define PPM_FAILSAFE_TIMEOUT_US 250000UL

void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 3000) {
    if (ppmCounter > 0) {
      lastPpmChannelCount = ppmCounter;
      lastPpmFrameMicros = now;
    }
    ppmCounter = 0;
  } else {
    if (ppmCounter < CHANNELS) {
      ppm[ppmCounter] = diff;
      ppmCounter++;
    }
  }
}

// RC failsafe: nilai channel dianggap tidak valid jika tidak ada frame PPM baru.
int readChannel(byte ch, int minVal = 1000, int maxVal = 2000, int defaultVal = 1500) {
  if (ch >= CHANNELS) return defaultVal;

  noInterrupts();
  unsigned long frameTime = lastPpmFrameMicros;
  byte channelCount = lastPpmChannelCount;
  int val = ppm[ch];
  interrupts();

  if (frameTime == 0) return defaultVal;
  if ((unsigned long)(micros() - frameTime) > PPM_FAILSAFE_TIMEOUT_US) return defaultVal;
  if (channelCount <= ch) return defaultVal;

  // Pertahankan rentang validasi pulse seperti kode asli: 800..2200 us.
  if (val >= 800 && val <= 2200) return val;
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

        if (serialCommand == 'W') {
          resetPIDController();
          Serial.println("[CMD] W -> waypoint control aktif. PID di-reset.");
        }
        
        if (serialCommand == 'A') {
          // FORMAT: A,<servo>,<motor_bawah>,<motor_depan_kiri>,<motor_depan_kanan>[,<dir_bawah>[,<dir_depan_kiri>,<dir_depan_kanan>]]
          int comma1 = serialInputBuffer.indexOf(',');
          int comma2 = serialInputBuffer.indexOf(',', comma1 + 1);
          int comma3 = serialInputBuffer.indexOf(',', comma2 + 1);
          int comma4 = serialInputBuffer.indexOf(',', comma2 > 0 && comma3 > 0 ? comma3 + 1 : -1);
          int comma5 = (comma4 > 0) ? serialInputBuffer.indexOf(',', comma4 + 1) : -1;
          int comma6 = (comma5 > 0) ? serialInputBuffer.indexOf(',', comma5 + 1) : -1;
          int comma7 = (comma6 > 0) ? serialInputBuffer.indexOf(',', comma6 + 1) : -1;

          // Cek apakah ada minimal 4 koma sebelum nilai diekstrak
          if (comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0) {
            String servoStr          = serialInputBuffer.substring(comma1 + 1, comma2);
            String motorBwhStr       = serialInputBuffer.substring(comma2 + 1, comma3);
            String motorDepanKiriStr = serialInputBuffer.substring(comma3 + 1, comma4);

            if (comma7 > 0) {
              // 7 koma: A,servo,motor_bwh,mot_d_l,mot_d_r,dir_bwh,dir_d_l,dir_d_r (arah motor depan independen)
              String motorDepanKananStr = serialInputBuffer.substring(comma4 + 1, comma5);
              String dirBwhStr          = serialInputBuffer.substring(comma5 + 1, comma6);
              String dirDepanKiriStr    = serialInputBuffer.substring(comma6 + 1, comma7);
              String dirDepanKananStr   = serialInputBuffer.substring(comma7 + 1);
              ai_motor_depan_kanan_val  = motorDepanKananStr.toInt();
              ai_dir_val                = dirBwhStr.toInt();
              ai_dir_depan_kiri_val     = dirDepanKiriStr.toInt();
              ai_dir_depan_kanan_val    = dirDepanKananStr.toInt();
            } else if (comma5 > 0) {
              // 5 koma: A,servo,motor_bwh,mot_d_l,mot_d_r,dir (semua motor pakai dir yang sama)
              String motorDepanKananStr = serialInputBuffer.substring(comma4 + 1, comma5);
              String dirStr             = serialInputBuffer.substring(comma5 + 1);
              ai_motor_depan_kanan_val  = motorDepanKananStr.toInt();
              ai_dir_val                = dirStr.toInt();
              ai_dir_depan_kiri_val     = dirStr.toInt();
              ai_dir_depan_kanan_val    = dirStr.toInt();
            } else {
              // 4 koma: A,servo,motor_bwh,mot_d_l,mot_d_r (default semua maju 1000)
              String motorDepanKananStr = serialInputBuffer.substring(comma4 + 1);
              ai_motor_depan_kanan_val  = motorDepanKananStr.toInt();
              ai_dir_val                = 1000; // Default maju
              ai_dir_depan_kiri_val     = 1000;
              ai_dir_depan_kanan_val    = 1000;
            }

            ai_servo_val             = servoStr.toInt();
            ai_motor_val             = motorBwhStr.toInt();
            ai_motor_depan_kiri_val  = motorDepanKiriStr.toInt();
          }
        } else if (serialCommand == 'C') {
          int comma = serialInputBuffer.indexOf(',');
          if (comma > 0) {
            String cmdAction = serialInputBuffer.substring(comma + 1);
            if (cmdAction == "INC") {
              counter++;
              if (counter > dataIndex) counter = dataIndex;
              is_new_wp = true;
              resetPIDController();
              bool stillInZone = (counter > uwStart && counter <= uwEnd) 
                              || (counter > surfStart && counter <= surfEnd);
              if (!stillInZone) {
                portraitState = PT_NORMAL;
              }
              Serial.print("[WP] Manual INC via Serial. Target WP #");
              Serial.println(counter);
            } else if (cmdAction == "DEC") {
              if (counter > 0) counter--;
              is_new_wp = true;
              resetPIDController();
            } else if (cmdAction == "RESET") {
              counter = 0;
              resetPIDController();
              portraitState = PT_NORMAL;
              dockingState = DK_IDLE;
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
              Serial.println("[WP] Saved to flash.");
            } else if (subCmd == "GET_WP") {
              Serial.println("SYNC_WP_START");
              for (int i = 0; i < dataIndex; i++) {
                Serial.print("SYNC_WP,");
                Serial.print(i);
                Serial.print(",");
                Serial.print(latitudes[i], 6);
                Serial.print(",");
                Serial.println(longitudes[i], 6);
                delay(1);
              }
              Serial.println("SYNC_WP_END");
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
            } else if (subCmd.startsWith("REPLACE,")) {
              int comma2 = subCmd.indexOf(',');
              int comma3 = subCmd.indexOf(',', comma2 + 1);
              int comma4 = subCmd.indexOf(',', comma3 + 1);
              if (comma2 > 0 && comma3 > 0 && comma4 > 0) {
                int idx = subCmd.substring(comma2 + 1, comma3).toInt();
                String latStr = subCmd.substring(comma3 + 1, comma4);
                String lonStr = subCmd.substring(comma4 + 1);
                if (idx >= 0 && idx < dataIndex) {
                  latitudes[idx] = latStr.toFloat();
                  longitudes[idx] = lonStr.toFloat();
                  saveDataToMemory();
                  Serial.println("Titik " + String(idx) + " berhasil direplace via Serial.");
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
        } else if (serialCommand == 'S') {
          // Settings command: S,PORTRAIT,... atau S,PT_RANGE,...
          int comma1 = serialInputBuffer.indexOf(',');
          if (comma1 > 0) {
            String subCmd = serialInputBuffer.substring(comma1 + 1);
            if (subCmd.startsWith("PORTRAIT,")) {
              // Format: S,PORTRAIT,speed,stopMs,reverseMs,reverseSpeed
              int c1 = subCmd.indexOf(',');
              int c2 = subCmd.indexOf(',', c1 + 1);
              int c3 = subCmd.indexOf(',', c2 + 1);
              int c4 = subCmd.indexOf(',', c3 + 1);
              if (c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0) {
                portraitSpeed = subCmd.substring(c1 + 1, c2).toInt();
                portraitStopMs = subCmd.substring(c2 + 1, c3).toInt();
                portraitReverseMs = subCmd.substring(c3 + 1, c4).toInt();
                portraitReverseSpeed = subCmd.substring(c4 + 1).toInt();
                Serial.println("Portrait Config: Spd=" + String(portraitSpeed) + 
                              " Stop=" + String(portraitStopMs) + "ms" +
                              " Rev=" + String(portraitReverseMs) + "ms" +
                              " RevSpd=" + String(portraitReverseSpeed));
              }
            } else if (subCmd.startsWith("PT_RANGE,")) {
              // Format: S,PT_RANGE,uwStart,uwEnd,surfStart,surfEnd
              int c1 = subCmd.indexOf(',');
              int c2 = subCmd.indexOf(',', c1 + 1);
              int c3 = subCmd.indexOf(',', c2 + 1);
              int c4 = subCmd.indexOf(',', c3 + 1);
              if (c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0) {
                uwStart = subCmd.substring(c1 + 1, c2).toInt();
                uwEnd = subCmd.substring(c2 + 1, c3).toInt();
                surfStart = subCmd.substring(c3 + 1, c4).toInt();
                surfEnd = subCmd.substring(c4 + 1).toInt();
                portraitState = PT_NORMAL;
                Serial.println("Portrait Range: UW=" + String(uwStart) + "-" + String(uwEnd) +
                              " Surf=" + String(surfStart) + "-" + String(surfEnd));
              }
            } else if (subCmd == "DOCK_EN" || subCmd == "DOCK_DIS") {
              // S,DOCK_EN  -> Aktifkan docking mission
              // S,DOCK_DIS -> Nonaktifkan docking mission (skip ke WP_COMPLETE saja)
              dockingEnabled = (subCmd == "DOCK_EN");
              if (!dockingEnabled) dockingState = DK_IDLE; // Reset jika dimatikan
              Serial.println("[DOCK] Docking " + String(dockingEnabled ? "ENABLED" : "DISABLED"));
            // === [DOCKING SWING LAMA - DINONAKTIFKAN] ===
            // } else if (subCmd == "DOCK_SWING") {
            //   if (dockingState == DK_IDLE && dockingEnabled) {
            //     dockingState = DK_SWING;
            //     dockingTimer = millis();
            //     Serial.println("[DOCK] Command DOCK_SWING diterima! Mengeksekusi manuver...");
            //   }
            } else if (subCmd.startsWith("DOCK,")) {
              // Format: S,DOCK,motorUtama,chargeMs,direction,servoLeft,servoRight
              int c1 = subCmd.indexOf(',');
              int c2 = subCmd.indexOf(',', c1 + 1);
              int c3 = subCmd.indexOf(',', c2 + 1);
              int c4 = subCmd.indexOf(',', c3 + 1);
              int c5 = subCmd.indexOf(',', c4 + 1);
              if (c1 > 0 && c2 > 0 && c3 > 0) {
                dockMotorUtama = subCmd.substring(c1 + 1, c2).toInt();
                dockChargeMs = subCmd.substring(c2 + 1, c3).toInt();
                if (c4 > 0) {
                  dockTurnDirection = subCmd.substring(c3 + 1, c4).toInt();
                  if (c5 > 0) {
                    dockServoLeft = subCmd.substring(c4 + 1, c5).toInt();
                    dockServoRight = subCmd.substring(c5 + 1).toInt();
                  } else {
                    dockServoLeft = subCmd.substring(c4 + 1).toInt();
                  }
                } else {
                  dockTurnDirection = subCmd.substring(c3 + 1).toInt();
                }
                dockingState = DK_IDLE; // Reset state jika config berubah
                Serial.println("Dock Config: Motor=" + String(dockMotorUtama) +
                              " Charge=" + String(dockChargeMs) + "ms" +
                              " Dir=" + String(dockTurnDirection) +
                              " Left=" + String(dockServoLeft) +
                              " Right=" + String(dockServoRight));
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

  // --- Inisialisasi GPS (ZED-F9P NMEA Parser @ 115200 Baud) ---
  Serial.println();
  Serial.println("========================================");
  Serial.println(" ESP32 GPS RECEIVER");
  Serial.println(" Input : ZED-F9P UART1");
  Serial.println(" Rate  : 5 Hz");
  Serial.println(" Baud  : 115200");
  Serial.println(" Lat/Lon : 8 digit decimal");
  Serial.println("========================================");
  GPS_SERIAL.setRxBufferSize(1024); // Perbesar buffer agar data 115200bps tidak tumpah
  GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX); 
  Serial.println("[GPS] NMEA Parser siap (115200 baud, tanpa library).");

  Wire.begin(21, 22); 
  Wire.setTimeOut(150); // Mencegah I2C blocking tak terhingga jika kena EMI

  // ========================================================
  // 3. INISIALISASI OLED DSP-0109
  // ========================================================
  if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledOk = true;
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.println("  NAVANTARA ASV");
    oled.println("  Booting...");
    oled.display();
    Serial.println("[OLED] DSP-0109 (SSD1306) berhasil diinisialisasi.");
  } else {
    Serial.println("[OLED] GAGAL: SSD1306 tidak ditemukan di 0x3C!");
  }

  // Inisialisasi PPM
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  loadDataFromMemory(); 

  Serial.println("");
  Serial.println("GPS + WAYPOINT SYSTEM (INTEGRATED)");
  Serial.println("================================");
  Serial.print("Data tersimpan: ");
  Serial.print(dataIndex);
  Serial.print("/");
  Serial.print(MAX_DATA);
  Serial.println(" titik");
  Serial.println("================================");
}

// --- Variabel Global Telemetri & Timing ---
float heading = 0.0;
double cog = 0.0; // Course Over Ground
unsigned long lastLoopTime = 0;

// ======================================================
// DATA NAVIGASI UNTUK OLED
// ======================================================
bool compassValidGlobal = false;
double wp_dist_m = 0.0;
double wp_target_brg = 0.0;
double wp_error_hdg = 0.0;

// Toleransi untuk dianggap sudah lurus terhadap arah waypoint
#define HEADING_STRAIGHT_TOLERANCE 3.0
// ---------------------------------

// ======================================================
// GET FIELD NMEA
// ======================================================
String getField(const String &data, int index) {
  int start = 0;
  int field = 0;
  for (int i = 0; i <= data.length(); i++) {
    if (i == data.length() || data.charAt(i) == ',') {
      if (field == index) {
        return data.substring(start, i);
      }
      field++;
      start = i + 1;
    }
  }
  return "";
}

// ======================================================
// NMEA COORDINATE CONVERTER
// ======================================================
double nmeaToDecimal(String value, String direction) {
  if (value.length() < 4) {
    return 0.0;
  }
  double raw = value.toDouble();
  int degrees = (int)(raw / 100.0);
  double minutes = raw - (degrees * 100.0);
  double result = degrees + (minutes / 60.0);
  if (direction == "S" || direction == "W") {
    result *= -1.0;
  }
  return result;
}

// ======================================================
// PARSE GGA
// ======================================================
void parseGGA(const String &sentence) {
  String latStr = getField(sentence, 2);
  String latDir = getField(sentence, 3);
  String lonStr = getField(sentence, 4);
  String lonDir = getField(sentence, 5);
  String quality = getField(sentence, 6);
  String sat = getField(sentence, 7);
  String hdopValue = getField(sentence, 8);
  String altitudeValue = getField(sentence, 9);

  if (latStr.length() > 0) {
    latitude = nmeaToDecimal(latStr, latDir);
  }
  if (lonStr.length() > 0) {
    longitude = nmeaToDecimal(lonStr, lonDir);
  }
  if (sat.length() > 0) {
    satellites = sat.toInt();
  }
  if (hdopValue.length() > 0) {
    hdop = hdopValue.toDouble();
  }
  if (altitudeValue.length() > 0) {
    altitude = altitudeValue.toDouble();
  }

  // --- FIX STATUS & LED INDICATION ---
  if (quality == "0") {
    fixStatus = "NO FIX";
    gpsFixType = 0;
    digitalWrite(LED_GPS, LOW);
  } else if (quality == "1") {
    fixStatus = "GPS";
    gpsFixType = 1;
    digitalWrite(LED_GPS, HIGH);
  } else if (quality == "2") {
    fixStatus = "DGPS";
    gpsFixType = 2;
    digitalWrite(LED_GPS, HIGH);
  } else if (quality == "4") {
    fixStatus = "RTK FIX";
    gpsFixType = 4;
    digitalWrite(LED_GPS, HIGH);
  } else if (quality == "5") {
    fixStatus = "RTK FLOAT";
    gpsFixType = 5;
    digitalWrite(LED_GPS, HIGH);
  } else if (quality == "6") {
    fixStatus = "DR";
    gpsFixType = 6;
    digitalWrite(LED_GPS, HIGH);
  } else {
    fixStatus = "UNKNOWN";
    gpsFixType = 0;
    digitalWrite(LED_GPS, LOW);
  }
}

// ======================================================
// PARSE RMC
// ======================================================
void parseRMC(const String &sentence) {
  String speedStr = getField(sentence, 7);
  if (speedStr.length() > 0) {
    double knots = speedStr.toDouble();
    speedKmh = knots * 1.852; // knots -> km/h
  }

  String cogStr = getField(sentence, 8);
  if (cogStr.length() > 0) {
    cog = cogStr.toDouble();
  }
}

// ======================================================
// PRINT DATA GPS KE SERIAL (8 Digit Decimal Precision)
// ======================================================
void printGPS() {
  Serial.print("GPS,");
  Serial.print(latitude, 8);
  Serial.print(",");
  Serial.print(longitude, 8);
  Serial.print(",");
  Serial.print(fixStatus);
  Serial.print(",");
  Serial.print(satellites);
  Serial.print(",");
  Serial.print(hdop, 2);
  Serial.print(",");
  Serial.print(speedKmh, 2);
  Serial.print(",");
  Serial.print(altitude, 2);
  Serial.print(",");
  Serial.println(gpsHz, 2);
}

// ======================================================
// PARSE NMEA ROUTER
// ======================================================
void parseNMEA(const String &sentence) {
  if (sentence.startsWith("$GNGGA") || sentence.startsWith("$GPGGA")) {
    parseGGA(sentence);

    unsigned long now = millis();
    if (lastGGA != 0) {
      unsigned long delta = now - lastGGA;
      if (delta > 0) {
        gpsHz = 1000.0 / (double)delta;
      }
    }
    lastGGA = now;
    printGPS();
  } else if (sentence.startsWith("$GNRMC") || sentence.startsWith("$GPRMC")) {
    parseRMC(sentence);
  }
}

// ======================================================
// BACA STREAM NMEA DARI ZED-F9P (Non-blocking)
// ======================================================
void readGPS_NMEA() {
  while (GPS_SERIAL.available()) {
    char c = GPS_SERIAL.read();
    if (c == '\n') {
      if (nmeaBuffer.length() > 0) {
        parseNMEA(nmeaBuffer);
      }
      nmeaBuffer = "";
    } else if (c != '\r') {
      nmeaBuffer += c;
    }
    if (nmeaBuffer.length() > 150) {
      nmeaBuffer = "";
    }
  }
}

// ============================================================
// Fungsi update OLED - indikator navigasi waypoint
// Update setiap 250ms, non-blocking
// ============================================================
void updateOLED() {
  if (!oledOk) return;
  if (millis() - lastOledUpdate < 250) return;
  lastOledUpdate = millis();

  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  // --------------------------------------------------------
  // BARIS 1: JUDUL
  // --------------------------------------------------------
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("=== NAVANTARA ASV ===");

  // --------------------------------------------------------
  // BARIS 2: WAYPOINT
  // counter menggunakan indeks array mulai dari 0.
  // Tampilan dibuat mulai dari WP 1 agar mudah dibaca operator.
  // --------------------------------------------------------
  oled.setTextSize(2);
  oled.setCursor(0, 13);

  if (dataIndex <= 0) {
    oled.print("WP: --/--");
  }
  else if (counter >= dataIndex) {
    oled.print("WP:");
    if (dataIndex < 10) oled.print("0");
    oled.print(dataIndex);
    oled.print("/");
    if (dataIndex < 10) oled.print("0");
    oled.print(dataIndex);
  }
  else {
    int displayWP = counter + 1;

    oled.print("WP:");
    if (displayWP < 10) oled.print("0");
    oled.print(displayWP);
    oled.print("/");
    if (dataIndex < 10) oled.print("0");
    oled.print(dataIndex);
  }

  // --------------------------------------------------------
  // BARIS 3: ERROR HEADING
  // 0 derajat = heading kapal tepat searah target waypoint.
  // --------------------------------------------------------
  oled.setTextSize(1);
  oled.setCursor(0, 35);
  oled.print("ERR:");

  if (!isManual && serialCommand == 'W' && compassValidGlobal && dataIndex > 0 && counter < dataIndex) {
    oled.print(wp_error_hdg, 1);
    oled.print((char)247);
  } else {
    oled.print("--.-");
    oled.print((char)247);
  }

  // --------------------------------------------------------
  // STATUS ARAH
  // --------------------------------------------------------
  oled.setCursor(70, 35);

  if (counter >= dataIndex && dataIndex > 0) {
    oled.print("SELESAI");
  }
  else if (isManual) {
    oled.print("MANUAL");
  }
  else if (serialCommand == 'A') {
    oled.print("AI");
  }
  else if (!compassValidGlobal) {
    oled.print("CMPS ERR");
  }
  else if (fabs(wp_error_hdg) <= HEADING_STRAIGHT_TOLERANCE) {
    oled.print("LURUS");
  }
  else if (wp_error_hdg > 0) {
    oled.print("BELOK >");
  }
  else {
    oled.print("BELOK <");
  }

  // --------------------------------------------------------
  // BARIS 4: STATUS GPS
  // --------------------------------------------------------
  oled.setCursor(0, 47);
  oled.print("GPS:");

  if (gpsFixType == 0) {
    oled.print(" NO FIX");
  }
  else if (gpsFixType == 1) {
    oled.print(" GPS");
  }
  else if (gpsFixType == 2) {
    oled.print(" DGPS");
  }
  else if (gpsFixType == 4) {
    oled.print(" RTK FIX");
  }
  else if (gpsFixType == 5) {
    oled.print(" RTK FLOAT");
  }
  else if (gpsFixType == 6) {
    oled.print(" DR");
  }
  else {
    oled.print(" UNKNOWN");
  }

  // --------------------------------------------------------
  // BARIS 5: HEADING CMPS12
  // --------------------------------------------------------
  oled.setCursor(0, 58);
  oled.print("HDG:");

  if (compassValidGlobal) {
    oled.print(heading, 1);
    oled.print((char)247);
  } else {
    oled.print(" ERROR");
  }

  oled.display();
}

void loop() {
  // 1. BACA SERIAL TERUS MENERUS TANPA HENTI (Mencegah Buffer Overflow)
  checkSerialInput(); 

  // 2. BACA GPS NMEA TERUS MENERUS (Mencegah Buffer Overflow pada 115200 baud)
  readGPS_NMEA();

  // 3. Update OLED setiap 500ms (non-blocking)
  updateOLED();

  // 4. Batasi kecepatan sensor & aktuator ke 20Hz (50ms)
  if (millis() - lastLoopTime < 50) {
    return;
  }
  lastLoopTime = millis();
  
  bool compassValid = readCompass(heading);
  compassValidGlobal = compassValid;

  static unsigned long lastCompassErrorLog = 0;
  if (!compassValid && millis() - lastCompassErrorLog >= 1000) {
    Serial.println("[CMPS12] ERROR: heading tidak valid. Navigasi waypoint ditahan.");
    lastCompassErrorLog = millis();
  }

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
  int finalDir = 1000;             // Default Maju (1000us)
  int finalDirDepanKiri = 1000;    // Default Maju (1000us)
  int finalDirDepanKanan = 1000;   // Default Maju (1000us)             
  
  int wp_target_idx = 0;
  wp_dist_m = 0.0;
  wp_target_brg = 0.0;
  wp_error_hdg = 0.0;

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
    finalDirDepanKiri = ch8;
    finalDirDepanKanan = ch8;

    if (ch6 > 1900) { 
      // Posisi 3 (Bawah) - Siap-siap merekam
      if (!wasInCaptureMode) {
        Serial.println("🟡 MODE REKAM: Siap merekam waypoint baru.");
        wasInCaptureMode = true;
        captureTriggered = false;
      }
    } else if (ch6 >= 1300 && ch6 <= 1700) { 
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
        if (targetCaptureIndex >= 0 && targetCaptureIndex < dataIndex) {
          // MODE REPLACE (Selalu diizinkan walau memori penuh)
          if (gpsFixType > 0) {
            latitudes[targetCaptureIndex] = lat;
            longitudes[targetCaptureIndex] = lon;
            saveDataToMemory();
            Serial.println("📍 Titik ke-" + String(targetCaptureIndex) + " diganti (Replace).");
            targetCaptureIndex = -1; // Reset target
          } else {
            Serial.println("❌ GPS belum lock. Tidak dapat mengganti data.");
          }
        } else {
          // MODE TAMBAH BARU (Hanya diizinkan jika memori belum penuh)
          if (dataIndex >= MAX_DATA) {
            Serial.println("⚠ Memori penuh. Tidak bisa menambah titik lagi.");
          } else {
            if (gpsFixType > 0) { 
              latitudes[dataIndex] = lat;
              longitudes[dataIndex] = lon;
              dataIndex++;
              saveDataToMemory();
              Serial.println("📍 Titik ke-" + String(dataIndex - 1) + " direkam.");
            } else {
              Serial.println("❌ GPS belum lock. Tidak dapat menambah data.");
            }
          }
        }
        captureTriggered = true;
      }
      wasInCaptureMode = false;
    } else if (ch6 < 1020) { 
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
      resetPIDController();
      portraitState = PT_NORMAL;
      dockingState = DK_IDLE;
    }

    mode = "AUTO";
    finalDir = 1000; // Default Arah Maju (1000us)

    // --- PORTRAIT ZONE DETECTION ---
    bool isInPortraitZone = (counter > uwStart && counter <= uwEnd) 
                         || (counter > surfStart && counter <= surfEnd);
    bool isAtPortraitEnd = (counter == uwEnd) || (counter == surfEnd);

    // === [DOCKING LAMA - DINONAKTIFKAN] ===
    // Logika docking sekarang sepenuhnya dikendalikan oleh Jetson melalui command 'A'.
    // ESP32 hanya melaporkan status "DK_TRACKING_AI" sebagai trigger untuk Jetson.
    //
    // if (dockingState == DK_SWING) {
    //   status = "DK_SWING";
    //   finalMotor = dockMotorUtama;
    //   finalDir = 1000;
    //   ... (logika swing lama) ...
    // }
    // else if (dockingState == DK_COMPLETE) {
    //   ... (logika complete lama) ...
    // }
    // === [AKHIR DOCKING LAMA] ===
    
    // === PORTRAIT STOP (Motor utama mati, motor depan boleh jika AI aktif) ===
    if (portraitState == PT_STOP) {
      status = "PT_STOP";
      finalMotor = 1000;
      finalDir = 1000;
      finalServo = 90;
      // Izinkan motor depan jika Jetson mendeteksi kotak
      if (serialCommand == 'A') {
        finalMotorDepanKiri = ai_motor_depan_kiri_val;
        finalMotorDepanKanan = ai_motor_depan_kanan_val;
      }
      if (millis() - portraitTimer >= portraitStopMs) {
        portraitState = PT_REVERSE;
        portraitTimer = millis();
        Serial.println("Portrait: Mulai mundur...");
      }
    }
    // === PORTRAIT REVERSE (Hanya di WP 12 & 14 setelah berhenti: Motor utama mundur, semua motor depan mati) ===
    else if (portraitState == PT_REVERSE) {
      status = "PT_REVERSE";
      finalMotor = portraitReverseSpeed;
      finalDir = 2000;  // MUNDUR KHUSUS DI TITIK 12 & 14 (2000us)
      finalDirDepanKiri = 2000;
      finalDirDepanKanan = 2000;
      finalServo = 90;
      finalMotorDepanKiri = 1000;
      finalMotorDepanKanan = 1000;
      if (millis() - portraitTimer >= portraitReverseMs) {
        counter++;
        is_new_wp = true;
        resetPIDController();
        portraitState = PT_NORMAL;
        finalDir = 1000;  // Kembali maju normal (1000us)
        finalDirDepanKiri = 1000;
        finalDirDepanKanan = 1000;
        Serial.print("Portrait selesai. Lanjut ke WP #");
        Serial.println(counter);
      }
    }
    // === AI VISION MODE ===
    else if (serialCommand == 'A') {
      finalServo = ai_servo_val;
      finalMotor = ai_motor_val;
      finalDir = ai_dir_val; // Mengikuti perintah serial (1000 = maju, 2000 = mundur)
      finalDirDepanKiri = ai_dir_depan_kiri_val;
      finalDirDepanKanan = ai_dir_depan_kanan_val;
      finalMotorDepanKiri = ai_motor_depan_kiri_val;
      finalMotorDepanKanan = ai_motor_depan_kanan_val;
      status = "AI_ACTIVE";
      
      // Di portrait zone: kunci motor utama agar tidak sentak saat switching W<->A (hanya jika maju normal)
      if (isInPortraitZone && ai_motor_val > 1000 && ai_dir_val == 1000) {
        finalMotor = portraitSpeed;
        portraitState = PT_SLOW;
        status = "PT_SLOW_AI";
      }
    } 
    else if (serialCommand == 'W') {
      status = "WAYPOINT";

      if (!compassValid) {
        finalServo = 90;
        finalMotor = 1000;
        finalMotorDepanKiri = 1000;
        finalMotorDepanKanan = 1000;
        status = "COMPASS_INVALID";
      }
      else if (dataIndex > 0 && gpsFixType > 0) { 
        
        if (counter >= dataIndex) { 
          wp_target_idx = dataIndex;

          // === DOCKING STATE MACHINE (FALLBACK SAAT DI TITIK AKHIR WP) ===
          if (!dockingEnabled) {
            // Docking dinonaktifkan: langsung anggap WP_COMPLETE, berhenti di titik akhir
            finalServo = 90;
            finalMotor = 1000;
            finalMotorDepanKiri = 1000;
            finalMotorDepanKanan = 1000;
            status = "WP_COMPLETE";
          } else {
            // Menunggu pemicu AI atau trigger kedekatan
            finalServo = 90;
            finalMotor = 1000;
            finalMotorDepanKiri = 1000;
            finalMotorDepanKanan = 1000;
            status = "DK_TRACKING_AI";
          }
        } else { 
          double targetLat = latitudes[counter];
          double targetLon = longitudes[counter];
          
          if (is_new_wp) {
            resetPIDController();
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

Serial.print("WP=");
Serial.print(counter);
Serial.print(" | DIST=");
Serial.print(dist, 2);
Serial.print(" | PATH=");
Serial.print(path_angle, 2);
Serial.print(" | LOS=");
Serial.print(los_correction, 2);
Serial.print(" | TARGET=");
Serial.print(targetBearing, 2);
Serial.print(" | HDG=");
Serial.print(heading, 2);
Serial.print(" | ERR=");
Serial.print(errorHeading, 2);
Serial.print(" | SERVO=");
Serial.println(servoPos);

          // CH6 = speed motor utama, dengan RC failsafe.
          int motorSpeed = readChannel(6);
          
          // Di portrait zone: kunci motor utama ke portraitSpeed
          if (isInPortraitZone) {
            motorSpeed = portraitSpeed;
            portraitState = PT_SLOW;
            status = "PT_SLOW";
          }
          
          finalMotor = motorSpeed;          
          
          finalMotorDepanKiri = 1000;       // Paksa mati di Mode W
          finalMotorDepanKanan = 1000;      // Paksa mati di Mode W

          if (dist < 1.30) {
            if (isAtPortraitEnd && portraitState == PT_SLOW) {
              // Sampai di titik akhir portrait -> STOP, TAHAN counter
              portraitState = PT_STOP;
              portraitTimer = millis();
              Serial.print("Portrait STOP di WP #");
              Serial.println(counter);
            } else {
              counter++;
              is_new_wp = true;
              resetPIDController();
              Serial.print("WP #");
              Serial.print(counter);
              Serial.println(" tercapai. Menuju WP berikutnya.");
              
              // Jika keluar dari portrait zone, reset state ke normal
              if (portraitState == PT_SLOW) {
                bool stillInZone = (counter > uwStart && counter <= uwEnd) 
                                || (counter > surfStart && counter <= surfEnd);
                if (!stillInZone) {
                  portraitState = PT_NORMAL;
                }
              }
            }
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
      } // selesai blok compassValid
    }

  // ========================================
  // --- 3. Kontrol Aktuator Lanjutan ---
  // ========================================

  // [NEW] Slew Rate Limiter (Soft Start) untuk memuluskan transisi PWM
  // Motor langsung dikirim tanpa ramping (realtime dari RC)

  servoKiri.write(finalServo); 
  servoKanan.write(finalServo); 

  dirDepanKiri.writeMicroseconds(finalDirDepanKiri);
  dirDepanKanan.writeMicroseconds(finalDirDepanKanan);
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
  jsonDoc["cmps_ok"] = compassValid;
  jsonDoc["cog"] = (float)round(cog * 10) / 10; // Kirim COG ke Jetson
  jsonDoc["lat"] = latitude;
  jsonDoc["lon"] = longitude;
  jsonDoc["spd"] = (float)round(speedKmh * 100) / 100;
  jsonDoc["sat"] = satellites;
  jsonDoc["fix"] = fixStatus;
  jsonDoc["hdop"] = (float)round(hdop * 100) / 100;
  jsonDoc["alt"] = (float)round(altitude * 10) / 10;
  jsonDoc["hz"] = (float)round(gpsHz * 10) / 10;

  jsonDoc["srv"] = finalServo;
  jsonDoc["mot"] = finalMotor;
  jsonDoc["m_dl"] = finalMotorDepanKiri;
  jsonDoc["m_dr"] = finalMotorDepanKanan;
  jsonDoc["ppm_ok"] = (lastPpmFrameMicros != 0 && (unsigned long)(micros() - lastPpmFrameMicros) <= PPM_FAILSAFE_TIMEOUT_US);
  
  if (serialCommand == 'A') {
    // Mode Auto: Data inversi telah dihapus untuk menghemat bandwidth
  }

  jsonDoc["w_id"] = counter;
  jsonDoc["w_tot"] = dataIndex;
  jsonDoc["p_st"] = (int)portraitState;  // 0=Normal, 1=Slow, 2=Stop, 3=Reverse
  jsonDoc["dk_st"] = (int)dockingState;  // 0=Idle, 1=Swing, 2=Complete

  if (mode == "AUTO" && serialCommand == 'W') { 
    if (status == "DK_COMPLETE" || status == "DK_SWING" || status == "DK_TRACKING_AI") {
      jsonDoc["w_dst"] = 0.0;
      jsonDoc["w_brg"] = 0.0;
      jsonDoc["w_err"] = 0.0;
      jsonDoc["xte"] = 0.0;
    } else if (status == "WP_COMPLETE") {
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

  // --- 4. Kirim Telemetri ke Jetson ---
  // Kita kirim pada kecepatan 50Hz (setiap 20ms) agar sangat responsif
  serializeJson(jsonDoc, Serial);
  Serial.println(); 
}