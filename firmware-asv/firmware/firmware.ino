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

// ---------------- GPS (Custom UBX Parser, TANPA LIBRARY!) ----------------
HardwareSerial gpsSerial(2); // Menggunakan Serial Port 2 ESP32
#define GPS_BAUD_RATE 115200 // Baud rate GPS (HARUS sudah disimpan di u-center!)
#define GPS_RX_PIN 16 // Pin RX untuk komunikasi GPS (default Serial 2 RX)
#define GPS_TX_PIN 17 // Pin TX untuk komunikasi GPS (default Serial 2 TX)

// --- UBX Parser State Machine ---
uint8_t ubxState = 0;
uint16_t ubxPayloadLength = 0;
uint16_t ubxPayloadCounter = 0;
uint8_t ubxPayload[100];
uint8_t ubxCkA = 0, ubxCkB = 0;
uint8_t gpsFixType = 0; // Variabel global pengganti myGPS.getFixType()

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
            } else if (subCmd == "DOCK_EN" || subCmd == "DOCK_DIS") {
              // S,DOCK_EN  -> Aktifkan docking mission
              // S,DOCK_DIS -> Nonaktifkan docking mission (skip ke WP_COMPLETE saja)
              dockingEnabled = (subCmd == "DOCK_EN");
              if (!dockingEnabled) dockingState = DK_IDLE; // Reset jika dimatikan
              Serial.println("[DOCK] Docking " + String(dockingEnabled ? "ENABLED" : "DISABLED"));
            } else if (subCmd == "DOCK_SWING") {
              if (dockingState == DK_IDLE && dockingEnabled) {
                dockingState = DK_SWING;
                dockingTimer = millis();
                Serial.println("[DOCK] Command DOCK_SWING diterima! Mengeksekusi manuver...");
              }
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

  // --- Inisialisasi GPS (Custom UBX Parser) ---
  // PENTING: Modul GPS HARUS sudah dikonfigurasi via u-center sebelumnya:
  //   - Baud rate: 115200
  //   - Output: UBX only (matikan NMEA)
  //   - Navigation Rate: 5Hz
  //   - GNSS: GPS + GLONASS
  //   - Simpan ke Flash (Send CFG-CFG Save)
  Serial.println("Mendengarkan GPS U-Blox (Custom UBX Parser)...");
  gpsSerial.setRxBufferSize(1024); // Perbesar buffer agar data 115200bps tidak tumpah
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 
  Serial.println("[GPS] Parser UBX-NAV-PVT siap (115200 baud, tanpa library).");

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
double lat = 0.0, lon = 0.0;
double speed = 0.0; 
double cog = 0.0; // Course Over Ground
int sats = 0;
unsigned long lastLoopTime = 0;
// ---------------------------------

// --- Custom UBX-NAV-PVT Parser ---
// Fungsi ini dipanggil otomatis saat 92-byte payload PVT berhasil divalidasi.
void parsePVT() {
  gpsFixType = ubxPayload[20];
  sats = ubxPayload[23];

  // Ekstraksi 4 byte gabungan (Little Endian) sesuai Data Sheet u-blox
  int32_t lon_raw    = ubxPayload[24] | (ubxPayload[25]<<8) | (ubxPayload[26]<<16) | (ubxPayload[27]<<24);
  int32_t lat_raw    = ubxPayload[28] | (ubxPayload[29]<<8) | (ubxPayload[30]<<16) | (ubxPayload[31]<<24);
  int32_t gSpeed_raw = ubxPayload[60] | (ubxPayload[61]<<8) | (ubxPayload[62]<<16) | (ubxPayload[63]<<24);
  int32_t head_raw   = ubxPayload[64] | (ubxPayload[65]<<8) | (ubxPayload[66]<<16) | (ubxPayload[67]<<24);

  if (gpsFixType > 0) {
    lat   = lat_raw / 10000000.0;
    lon   = lon_raw / 10000000.0;
    speed = gSpeed_raw / 1000.0 * 3.6; // m/s -> km/h
    cog   = head_raw / 100000.0;       // 1e-5 deg -> deg
    digitalWrite(LED_GPS, HIGH);
  } else {
    lat = 0.0; lon = 0.0; speed = 0.0; cog = 0.0; sats = 0;
    digitalWrite(LED_GPS, LOW);
  }
}

// --- Mesin Penyadap UBX (dipanggil di awal loop) ---
void readGPS_UBX() {
  while (gpsSerial.available()) {
    uint8_t c = gpsSerial.read();
    switch (ubxState) {
      case 0: if (c == 0xB5) ubxState = 1; break;
      case 1: if (c == 0x62) ubxState = 2; else ubxState = 0; break;
      case 2: if (c == 0x01) { ubxState = 3; ubxCkA = c; ubxCkB = c; } else ubxState = 0; break;
      case 3: if (c == 0x07) { ubxState = 4; ubxCkA += c; ubxCkB += ubxCkA; } else ubxState = 0; break;
      case 4: ubxPayloadLength = c; ubxCkA += c; ubxCkB += ubxCkA; ubxState = 5; break;
      case 5: ubxPayloadLength |= (c << 8); ubxCkA += c; ubxCkB += ubxCkA;
              if (ubxPayloadLength == 92) { ubxState = 6; ubxPayloadCounter = 0; }
              else ubxState = 0; break;
      case 6: ubxPayload[ubxPayloadCounter++] = c;
              ubxCkA += c; ubxCkB += ubxCkA;
              if (ubxPayloadCounter == 92) ubxState = 7;
              break;
      case 7: if (c == ubxCkA) ubxState = 8; else ubxState = 0; break;
      case 8: if (c == ubxCkB) parsePVT();
              ubxState = 0; break;
    }
  }
}

// ============================================================
// Fungsi update OLED — dipanggil di dalam loop() setiap 500ms
// ============================================================
void updateOLED() {
  if (!oledOk) return;
  if (millis() - lastOledUpdate < 500) return;
  lastOledUpdate = millis();

  oled.clearDisplay();

  // --- Baris 1: Judul ---
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("=== NAVANTARA ASV ===");

  // --- Baris 2: HDG (Heading Kompas) ---
  // Tampil besar di tengah layar agar mudah dibaca dari jauh
  oled.setTextSize(2);
  oled.setCursor(0, 14);
  oled.print("HDG ");
  oled.print((int)round(heading));
  oled.println((char)247); // Karakter derajat °

  // --- Baris 3 & 4: COG & Satelit (font kecil) ---
  oled.setTextSize(1);
  oled.setCursor(0, 36);
  oled.print("COG: ");
  oled.print(cog, 1);
  oled.println((char)247);

  oled.setCursor(0, 48);
  oled.print("SAT: ");
  oled.print(sats);
  // Tampilkan status fix GPS agar mudah dimonitor lapangan
  if (gpsFixType >= 3) {
    oled.print(" [FIX 3D]");
  } else if (gpsFixType == 2) {
    oled.print(" [FIX 2D]");
  } else {
    oled.print(" [NO FIX]");
  }

  oled.display();
}

void loop() {
  // 1. BACA SERIAL TERUS MENERUS TANPA HENTI (Mencegah Buffer Overflow)
  checkSerialInput(); 

  // 2. BACA GPS UBX TERUS MENERUS (Mencegah Buffer Overflow pada 115200 baud)
  readGPS_UBX();

  // 3. Update OLED setiap 500ms (non-blocking)
  updateOLED();

  // 4. Batasi kecepatan sensor & aktuator ke 20Hz (50ms)
  if (millis() - lastLoopTime < 50) {
    return;
  }
  lastLoopTime = millis();
  
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
  int finalDir = 1000;             // Default Maju (1000us)             
  
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
    finalDir = 1000; // Default Arah Maju (1000us)

    // --- PORTRAIT ZONE DETECTION ---
    bool isInPortraitZone = (counter > uwStart && counter <= uwEnd) 
                         || (counter > surfStart && counter <= surfEnd);
    bool isAtPortraitEnd = (counter == uwEnd) || (counter == surfEnd);

    // === DOCKING SWING (Fase Manuver Ayunan ke Dermaga - Prioritas Tertinggi) ===
    if (dockingState == DK_SWING) {
      status = "DK_SWING";
      finalMotor = dockMotorUtama; // Gas dorong maju
      finalDir = 1000;             // Arah maju
      
      if (dockTurnDirection == 0) {
        // Arena A: Mengincar bola kiri. Buritan ke Kanan. Servo dipatahkan ke KIRI (dockServoLeft)
        finalServo = dockServoLeft; 
      } else {
        // Arena B: Mengincar bola kanan. Buritan ke Kiri. Servo dipatahkan ke KANAN (dockServoRight)
        finalServo = dockServoRight;
      }
      
      finalMotorDepanKiri = 1000;
      finalMotorDepanKanan = 1000;

      if (millis() - dockingTimer >= dockChargeMs) {
        dockingState = DK_COMPLETE;
        Serial.println("[DOCK] DOCKING SWING COMPLETE! Matikan mesin.");
      }
    }
    // === DOCKING COMPLETE (Fase Mesin Mati Total di Dermaga) ===
    else if (dockingState == DK_COMPLETE) {
      finalServo = 90;
      finalMotor = 1000;
      finalDir = 1000;
      finalMotorDepanKiri = 1000;
      finalMotorDepanKanan = 1000;
      status = "DK_COMPLETE";
    }
    // === PORTRAIT STOP (Motor utama mati, motor depan boleh jika AI aktif) ===
    else if (portraitState == PT_STOP) {
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
      finalServo = 90;
      finalMotorDepanKiri = 1000;
      finalMotorDepanKanan = 1000;
      if (millis() - portraitTimer >= portraitReverseMs) {
        counter++;
        is_new_wp = true;
        portraitState = PT_NORMAL;
        finalDir = 1000;  // Kembali maju normal (1000us)
        Serial.print("Portrait selesai. Lanjut ke WP #");
        Serial.println(counter);
      }
    }
    // === AI VISION MODE ===
    else if (serialCommand == 'A') {
      finalServo = ai_servo_val;
      finalMotor = ai_motor_val;
      finalDir = 1000; // Maju normal
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
      
      if (dataIndex > 0 && gpsFixType > 0) { 
        
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
  // Motor langsung dikirim tanpa ramping (realtime dari RC)

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