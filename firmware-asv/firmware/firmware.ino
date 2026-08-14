#include <Wire.h> // Library untuk komunikasi I2C (untuk CMPS12)
#include <SparkFun_Ublox_Arduino_Library.h> // Library untuk GPS U-Blox
#include <ESP32Servo.h> // Library untuk mengontrol Servo dan ESC
#include <Preferences.h> // Library untuk menyimpan data di memori non-volatile (NVS/EEPROM)
#include <ArduinoJson.h> // Library untuk memproses dan mengirim data JSON (telemetri)

// ---------------- GPS (Diganti ke U-Blox 10Hz) ----------------
SFE_UBLOX_GPS myGPS;
HardwareSerial gpsSerial(2); // Menggunakan Serial Port 2 ESP32
#define GPS_BAUD_RATE 9600 // Baud rate GPS (hardware-locked di modul U-Blox, jangan diubah)
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
enum DockingState { DK_IDLE, DK_TURNING, DK_CHARGING, DK_COMPLETE };
DockingState dockingState = DK_IDLE;
unsigned long dockingTimer = 0;
float dockingInitialHeading = 0.0;
float dockingTargetHeading = 0.0;

// Konfigurasi Docking (Default, bisa diubah dari GUI via Jetson)
int dockMotorUtama = 1200;          // PWM motor utama saat docking
int dockMotorDepan = 1400;          // PWM motor depan pembantu belok
unsigned long dockChargeMs = 3000;  // Durasi maju menabrak dock (ms)
int dockHeadingTolerance = 5;       // Toleransi heading (derajat)
int dockTurnDirection = 0;          // 0=KIRI (Arena A), 1=KANAN (Arena B)

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

// ---------------- Baca heading CMPS12 ----------------
float readCompass() {
  static float last_valid_heading = 0.0;
  Wire.beginTransmission(CMPS12_ADDRESS);
  Wire.write(ANGLE_16BIT_REGISTER);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    // [FIX-2] Hanya return nilai terakhir yang valid. JANGAN reset Wire bus
    // di dalam loop 50Hz karena menyebabkan memory leak / crash ESP32.
    // Wire.setTimeOut(150) di setup() sudah cukup untuk mencegah blocking.
    return last_valid_heading;
  }

  // Semua argumen di-cast ke uint8_t agar cocok persis dengan overload:
  // uint8_t requestFrom(uint8_t address, uint8_t size, uint8_t sendStop)
  uint8_t bytesReceived = Wire.requestFrom((uint8_t)CMPS12_ADDRESS, (uint8_t)2, (uint8_t)1);
  if (bytesReceived == 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    unsigned int angle16 = (highByte << 8) | lowByte; 
    last_valid_heading = angle16 / 10.0;
    return last_valid_heading; 
  }
  return last_valid_heading; // Fallback ke heading terakhir
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
              displayAllData();
            } else if (subCmd == "GET_WP") {
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
            } else if (subCmd.startsWith("DOCK,")) {
              // Format: S,DOCK,motorUtama,motorDepan,chargeMs,tolerance,direction
              int c1 = subCmd.indexOf(',');
              int c2 = subCmd.indexOf(',', c1 + 1);
              int c3 = subCmd.indexOf(',', c2 + 1);
              int c4 = subCmd.indexOf(',', c3 + 1);
              int c5 = subCmd.indexOf(',', c4 + 1);
              if (c1 > 0 && c2 > 0 && c3 > 0 && c4 > 0 && c5 > 0) {
                dockMotorUtama = subCmd.substring(c1 + 1, c2).toInt();
                dockMotorDepan = subCmd.substring(c2 + 1, c3).toInt();
                dockChargeMs = subCmd.substring(c3 + 1, c4).toInt();
                dockHeadingTolerance = subCmd.substring(c4 + 1, c5).toInt();
                dockTurnDirection = subCmd.substring(c5 + 1).toInt();
                dockingState = DK_IDLE; // Reset state jika config berubah
                Serial.println("Dock Config: Motor=" + String(dockMotorUtama) +
                              " Depan=" + String(dockMotorDepan) +
                              " Charge=" + String(dockChargeMs) + "ms" +
                              " Tol=" + String(dockHeadingTolerance) + "deg" +
                              " Dir=" + String(dockTurnDirection));
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
  myGPS.setNavigationFrequency(5); // [FIX-1 REVISED] Diturunkan dari 10Hz -> 5Hz agar payload tidak overflow buffer UART 9600bps
  myGPS.setAutoPVT(true); 

  Wire.begin(21, 22); 
  Wire.setTimeOut(150); // Mencegah I2C blocking tak terhingga jika kena EMI

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

// --- Variabel Global Telemetri & Timing ---
float heading = 0.0;
double lat = 0.0, lon = 0.0;
double speed = 0.0; 
double cog = 0.0; // [NEW] Course Over Ground
int sats = 0;
unsigned long lastLoopTime = 0;
// ---------------------------------

void loop() {
  // 1. BACA SERIAL TERUS MENERUS TANPA HENTI (Mencegah Buffer Overflow)
  checkSerialInput(); 

  // 2. Batasi kecepatan sensor & aktuator ke 50Hz (20ms) agar I2C tidak macet
  if (millis() - lastLoopTime < 20) {
    return;
  }
  lastLoopTime = millis();

  // --- 3. Baca Sensor GPS ---
  if (myGPS.getPVT()) { 
    uint8_t fixType = myGPS.getFixType(); 

    if (fixType > 0) digitalWrite(LED_GPS, HIGH);
    else digitalWrite(LED_GPS, LOW);

    if (fixType > 0) { 
        lat = myGPS.getLatitude() / 10000000.0;
        lon = myGPS.getLongitude() / 10000000.0;
        speed = myGPS.getGroundSpeed() / 1000.0 * 3.6; 
        sats = myGPS.getSIV(); 
        cog = myGPS.getHeading() / 100000.0; // [NEW] Ambil Course Over Ground
    } else { 
        lat = 0.0;
        lon = 0.0;
        speed = 0.0;
        cog = 0.0;
        sats = 0;
    }
  }
  
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
        if (targetCaptureIndex >= 0 && targetCaptureIndex < dataIndex) {
          // MODE REPLACE (Selalu diizinkan walau memori penuh)
          if (myGPS.getFixType() > 0) {
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
            if (myGPS.getFixType() > 0) { 
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
      portraitState = PT_NORMAL;
      dockingState = DK_IDLE;
    }

    mode = "AUTO";
    finalDir = 1500; 

    // --- PORTRAIT ZONE DETECTION ---
    bool isInPortraitZone = (counter > uwStart && counter <= uwEnd) 
                         || (counter > surfStart && counter <= surfEnd);
    bool isAtPortraitEnd = (counter == uwEnd) || (counter == surfEnd);

    // === PORTRAIT STOP (Motor utama mati, motor depan boleh jika AI aktif) ===
    if (portraitState == PT_STOP) {
      status = "PT_STOP";
      finalMotor = 1000;
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
    // === PORTRAIT REVERSE (Motor utama mundur, semua motor depan mati) ===
    else if (portraitState == PT_REVERSE) {
      status = "PT_REVERSE";
      finalMotor = portraitReverseSpeed;
      finalDir = 1000;  // MUNDUR (menggunakan mekanisme DIR yang sudah ada)
      finalServo = 90;
      if (millis() - portraitTimer >= portraitReverseMs) {
        counter++;
        is_new_wp = true;
        portraitState = PT_NORMAL;
        finalDir = 1500;  // Kembali maju
        Serial.print("Portrait selesai. Lanjut ke WP #");
        Serial.println(counter);
      }
    }
    // === AI VISION MODE ===
    else if (serialCommand == 'A') {
      finalServo = ai_servo_val;
      finalMotor = ai_motor_val;
      finalDir = 1500;
      finalMotorDepanKiri = ai_motor_depan_kiri_val;
      finalMotorDepanKanan = ai_motor_depan_kanan_val;
      status = "AI_ACTIVE";
      
      // Di portrait zone: kunci motor utama agar tidak sentak saat switching W<->A
      if (isInPortraitZone) {
        finalMotor = portraitSpeed;
        portraitState = PT_SLOW;
        status = "PT_SLOW_AI";
      }
    } 
    else if (serialCommand == 'W') {
      status = "WAYPOINT";
      
      if (dataIndex > 0 && myGPS.getFixType() > 0) { 
        
        if (counter >= dataIndex) { 
          wp_target_idx = dataIndex;

          // === DOCKING STATE MACHINE ===
          if (dockingState == DK_IDLE) {
            // Fase 0: Catat heading awal, hitung target 90°
            dockingInitialHeading = heading;
            if (dockTurnDirection == 0) {
              // Arena A: Belok KIRI (-90)
              dockingTargetHeading = fmod((dockingInitialHeading - 90.0 + 360.0), 360.0);
            } else {
              // Arena B: Belok KANAN (+90)
              dockingTargetHeading = fmod((dockingInitialHeading + 90.0), 360.0);
            }
            dockingState = DK_TURNING;
            Serial.print("DOCKING START! Initial: ");
            Serial.print(dockingInitialHeading, 1);
            Serial.print(" -> Target: ");
            Serial.print(dockingTargetHeading, 1);
            Serial.print(" Dir: ");
            Serial.println(dockTurnDirection == 0 ? "KIRI" : "KANAN");
          }

          if (dockingState == DK_TURNING) {
            // Fase 1: Putar menuju target heading
            float dockError = dockingTargetHeading - heading;
            if (dockError > 180) dockError -= 360;
            if (dockError < -180) dockError += 360;

            if (abs(dockError) < dockHeadingTolerance) {
              // Heading tercapai → pindah ke CHARGING
              dockingState = DK_CHARGING;
              dockingTimer = millis();
              Serial.println("DOCKING: Heading tercapai! Mulai CHARGE ke dock...");
            } else {
              // Hitung servo dengan PID (reuse fungsi yang sudah ada)
              finalServo = PID_servo(dockingTargetHeading, heading);
              finalMotor = dockMotorUtama;
              finalDir = 1500; // Maju

              // Motor depan: dorong hidung ke arah putaran
              if (dockError > 0) {
                // Perlu belok KANAN → Depan Kiri ON
                finalMotorDepanKiri = dockMotorDepan;
                finalMotorDepanKanan = 1000;
              } else {
                // Perlu belok KIRI → Depan Kanan ON
                finalMotorDepanKiri = 1000;
                finalMotorDepanKanan = dockMotorDepan;
              }
              status = "DK_TURNING";
            }
          }

          if (dockingState == DK_CHARGING) {
            // Fase 2: Maju lurus menabrak dock
            finalServo = PID_servo(dockingTargetHeading, heading); // Tetap jaga heading lurus
            finalMotor = dockMotorUtama;
            finalDir = 1500;
            finalMotorDepanKiri = 1000;
            finalMotorDepanKanan = 1000;
            status = "DK_CHARGING";

            if (millis() - dockingTimer >= dockChargeMs) {
              dockingState = DK_COMPLETE;
              Serial.println("DOCKING COMPLETE! Kapal berhasil docking.");
            }
          }

          if (dockingState == DK_COMPLETE) {
            // Fase 3: Berhenti total
            finalServo = 90;
            finalMotor = 1000;
            finalMotorDepanKiri = 1000;
            finalMotorDepanKanan = 1000;
            status = "DK_COMPLETE";
          }
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
          
          // Di portrait zone: kunci motor utama ke portraitSpeed
          if (isInPortraitZone) {
            motorSpeed = portraitSpeed;
            portraitState = PT_SLOW;
            status = "PT_SLOW";
          }
          
          finalMotor = motorSpeed;          
          
          finalMotorDepanKiri = 1000;       // Paksa mati di Mode W
          finalMotorDepanKanan = 1000;      // Paksa mati di Mode W

          if (dist < 1.75) {
            if (isAtPortraitEnd && portraitState == PT_SLOW) {
              // Sampai di titik akhir portrait -> STOP, TAHAN counter
              portraitState = PT_STOP;
              portraitTimer = millis();
              Serial.print("Portrait STOP di WP #");
              Serial.println(counter);
            } else {
              counter++; 
              is_new_wp = true;
              Serial.print("WP #");
              Serial.print(counter);
              Serial.println(" tercapai. Menuju WP berikutnya.");
              
              // Jika keluar dari portrait zone, reset state ke normal
              if (portraitState == PT_SLOW) {
                bool stillInZone = (counter >= uwStart && counter <= uwEnd) 
                                || (counter >= surfStart && counter <= surfEnd);
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
    }
  }

  // ========================================
  // --- 3. Kontrol Aktuator Lanjutan ---
  // ========================================

  // [NEW] Slew Rate Limiter (Soft Start) untuk memuluskan transisi PWM
  static int currentMotorBawah = 1000;
  static int currentMotorDepanKiri = 1000;
  static int currentMotorDepanKanan = 1000;

  int max_step = 3; // [FIX-3] Diturunkan dari 15 -> 3 untuk soft-start yang lebih halus, menekan EMI spike inrush current

  // Soft-start Motor Utama (Bawah)
  if (finalMotor > currentMotorBawah + max_step) currentMotorBawah += max_step;
  else if (finalMotor < currentMotorBawah - max_step) currentMotorBawah -= max_step;
  else currentMotorBawah = finalMotor;
  finalMotor = currentMotorBawah;

  // Soft-start Motor Kiri
  if (finalMotorDepanKiri > currentMotorDepanKiri + max_step) currentMotorDepanKiri += max_step;
  else if (finalMotorDepanKiri < currentMotorDepanKiri - max_step) currentMotorDepanKiri -= max_step;
  else currentMotorDepanKiri = finalMotorDepanKiri;
  finalMotorDepanKiri = currentMotorDepanKiri;

  // Soft-start Motor Kanan
  if (finalMotorDepanKanan > currentMotorDepanKanan + max_step) currentMotorDepanKanan += max_step;
  else if (finalMotorDepanKanan < currentMotorDepanKanan - max_step) currentMotorDepanKanan -= max_step;
  else currentMotorDepanKanan = finalMotorDepanKanan;
  finalMotorDepanKanan = currentMotorDepanKanan;

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
  jsonDoc["cog"] = (float)round(cog * 10) / 10; // [NEW] Kirim COG ke Jetson
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
  jsonDoc["p_st"] = (int)portraitState;  // 0=Normal, 1=Slow, 2=Stop, 3=Reverse
  jsonDoc["dk_st"] = (int)dockingState;  // 0=Idle, 1=Turning, 2=Charging, 3=Complete

  if (mode == "AUTO" && serialCommand == 'W') { 
    if (status == "DK_COMPLETE" || status == "DK_TURNING" || status == "DK_CHARGING") {
      jsonDoc["w_dst"] = 0.0;
      jsonDoc["w_brg"] = (float)round(dockingTargetHeading * 100) / 100;
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