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

// ---------------- LED GPS FIX (TAMBAHAN) ----------------
#define LED_GPS 2   // LED ESP32 (GPIO 2) untuk indikasi lock GPS

// ---------------- CMPS12 ----------------
#define CMPS12_ADDRESS 0x60 // Alamat I2C Kompas CMPS12
#define ANGLE_16BIT_REGISTER 2 // Register untuk membaca heading 16-bit

// ---------------- Servo dan ESC ----------------
Servo rudderServo; // Objek untuk kemudi (Rudder)
Servo motorESC; // Objek untuk kontrol motor (ESC)
Servo auxOutput; // Objek untuk output tambahan (misal: pompa, lampu, dll)

// ---------------- PID ----------------
double Kp = 2.0, Ki = 0.0, Kd = 0.5; // Konstanta PID (Proporsional, Integral, Derivatif)

// --- Waypoint Inversion Control for AI Mode (MODIFIKASI) ---
// Waypoint index (0-indexed) where AI servo output inversion begins.
// E.g., setting this to 5 means inversion starts when targeting Waypoint #6.
// Ubah angka 5 menjadi indeks Waypoint yang Anda inginkan.
const int AI_SERVO_INVERSION_INDEX = 7; 

double error, lastError = 0, integral = 0; // Variabel perhitungan PID

// ---------------- Waypoint ----------------
#define MAX_DATA 20 // Batas maksimum jumlah Waypoint yang bisa disimpan
Preferences preferences; // Objek untuk manajemen memori NVS

float latitudes[MAX_DATA]; // Array untuk menyimpan lintang Waypoint
float longitudes[MAX_DATA]; // Array untuk menyimpan bujur Waypoint
int dataIndex = 0; // Jumlah Waypoint yang saat ini tersimpan
int counter = 0; // Indeks Waypoint yang sedang dituju (Target WP)

bool captureTriggered = false; // Flag apakah mode rekam sudah selesai menekan
bool wasInCaptureMode = false; // Flag status apakah sedang berada di posisi tengah CH6
bool wasInSaveMode = false; // Flag status apakah sudah menekan save (CH6 posisi rendah)

// --- KONTROL DARI JETSON/KOMUNIKASI SERIAL ---
char serialCommand = 'W'; // Perintah default: 'W' (Waypoint), 'A' (AI Control)
int ai_servo_val = 90; // Nilai servo dari perintah AI (0-180 derajat)
int ai_motor_val = 1500; // Nilai motor dari perintah AI (1000-2000 us)

// --- Buffer JSON Global ---
StaticJsonDocument<300> jsonDoc; // Dokumen JSON untuk telemetri

// --- PEMBARUAN: Buffer Global untuk Serial Non-Blocking ---
String serialInputBuffer = ""; // Buffer untuk menampung karakter serial yang masuk

// ---------------- Haversine ----------------
// Fungsi menghitung jarak antara dua koordinat dalam meter (R = jari-jari bumi)
#define R 6371000.0
double haversine(double lat1, double lon1, double lat2, double lon2) {
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a = sin(dLat / 2) * sin(dLat / 2) +
              cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c; // Mengembalikan jarak dalam meter
}

// ---------------- Bearing ----------------
// Fungsi menghitung arah (bearing) dari titik 1 ke titik 2 (0-360 derajat)
double bearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double dLon = radians(lon2 - lon1);
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double brng = atan2(y, x);
  return fmod((degrees(brng) + 360.0), 360.0); // Mengembalikan bearing dalam derajat (0-360)
}

// ---------------- Baca heading CMPS12 ----------------
// Fungsi membaca nilai heading dari kompas CMPS12 melalui I2C
float readCompass() {
  Wire.beginTransmission(CMPS12_ADDRESS);
  Wire.write(ANGLE_16BIT_REGISTER);
  Wire.endTransmission();

  Wire.requestFrom(CMPS12_ADDRESS, 2); // Meminta 2 byte data
  if (Wire.available() == 2) {
    byte highByte = Wire.read();
    byte lowByte = Wire.read();
    unsigned int angle16 = (highByte << 8) | lowByte; // Menggabungkan byte
    return angle16 / 10.0; // Heading dalam 0.1 derajat, dikonversi ke derajat
  }
  return -1; // Mengembalikan -1 jika gagal baca
}

// ---------------- PID untuk servo ----------------
// Fungsi menghitung output servo berdasarkan heading saat ini dan target bearing
int PID_servo(double setpoint, double input) {
  error = input - setpoint; // Error = Heading saat ini - Target Bearing

  // Menyesuaikan error untuk jalur terpendek (misal: 350 vs 10 derajat)
  if (error > 180) error -= 360; 
  if (error < -180) error += 360;

  integral += error; // Akumulasi error (Integral)
  double derivative = error - lastError; // Perubahan error (Derivatif)
  lastError = error; // Simpan error saat ini untuk perhitungan derivatif berikutnya

  double output = Kp * error + Ki * integral + Kd * derivative; // Output PID
  int servoPos = 90 + output; // Posisi Servo (90 adalah netral)

  // Batasan output servo
  if (servoPos > 180) servoPos = 180;
  if (servoPos < 0) servoPos = 0;

  return servoPos; // Mengembalikan nilai PWM servo (0-180 derajat)
}

// ---------------- PPM INPUT ----------------
#define PPM_PIN 4 // Pin untuk menerima sinyal PPM dari receiver
#define CHANNELS 10 // Jumlah maksimum channel yang didukung
volatile int ppm[CHANNELS]; // Array untuk menyimpan nilai pulse width (1000-2000 us)
volatile byte ppmCounter = 0; // Penghitung channel
volatile unsigned long lastMicros = 0; // Waktu terakhir sinyal PPM terdeteksi

// ISR (Interrupt Service Routine) untuk membaca sinyal PPM
void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 3000) { // Jika jeda terlalu lama, berarti sinyal baru dimulai
    ppmCounter = 0;
  } else {
    if (ppmCounter < CHANNELS) {
      ppm[ppmCounter] = diff; // Simpan lebar pulsa (nilai channel)
      ppmCounter++;
    }
  }
}

// Fungsi untuk membaca nilai channel PPM yang sudah difilter
int readChannel(byte ch, int minVal = 1000, int maxVal = 2000, int defaultVal = 1500) {
  if (ch < CHANNELS) {
    int val = ppm[ch];
    if (val >= 800 && val <= 2200) return val; // Filter nilai yang valid
  }
  return defaultVal; // Mengembalikan nilai default jika sinyal hilang/tidak valid
}

// ---------------- Fungsi Manajemen Data GPS ----------------
// Menyimpan semua Waypoint dari array ke memori NVS
void saveDataToMemory() {
  preferences.begin("gps-data", false); // Mulai sesi tulis
  preferences.putUInt("dataCount", dataIndex);
  for (int i = 0; i < dataIndex; i++) {
    String latKey = "lat" + String(i);
    String lngKey = "lng" + String(i);
    preferences.putFloat(latKey.c_str(), latitudes[i]);
    preferences.putFloat(lngKey.c_str(), longitudes[i]);
  }
  preferences.end(); // Akhiri sesi tulis
}

// Memuat semua Waypoint dari memori NVS ke array
void loadDataFromMemory() {
  preferences.begin("gps-data", true); // Mulai sesi baca
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
  preferences.end(); // Akhiri sesi baca
}

// Menghapus semua data Waypoint dari memori NVS
void clearAllData() {
  preferences.begin("gps-data", false);
  preferences.clear();
  preferences.end();
  dataIndex = 0;
  Serial.println("🗑 Semua data lama telah dihapus.");
}

// Menampilkan semua Waypoint yang tersimpan melalui Serial Monitor
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

// --- [PEMBARUAN] FUNGSI NON-BLOCKING UNTUK MEMBACA PERINTAH SERIAL ---
// Fungsi ini membaca perintah dari Jetson secara karakter per karakter (non-blocking)
void checkSerialInput() {
  // Loop selama ada data yang tersedia di buffer serial
  while (Serial.available() > 0) {
    char incomingChar = Serial.read(); // Baca 1 karakter
    
    // Jika karakter adalah newline ('\n'), ini menandakan akhir perintah
    if (incomingChar == '\n') {
      
      serialInputBuffer.trim(); // Hapus spasi di awal/akhir
      
      if (serialInputBuffer.length() > 0) {
        serialCommand = serialInputBuffer.charAt(0); // Ambil perintah utama ('W' atau 'A')
        
        // Memproses perintah AI (A,<servo_val>,<motor_val>)
        if (serialCommand == 'A') {
          int firstComma = serialInputBuffer.indexOf(',');
          int secondComma = serialInputBuffer.indexOf(',', firstComma + 1);

          if (firstComma > 0 && secondComma > 0) {
            String servoStr = serialInputBuffer.substring(firstComma + 1, secondComma);
            String motorStr = serialInputBuffer.substring(secondComma + 1);
            ai_servo_val = servoStr.toInt();
            ai_motor_val = motorStr.toInt();
          }
        }
      }
      
      // Kosongkan buffer setelah perintah selesai diproses
      serialInputBuffer = "";
    } else {
      // Jika bukan newline, tambahkan karakter ke buffer
      if (serialInputBuffer.length() < 128) { // Batas 128 karakter untuk mencegah overflow
        serialInputBuffer += incomingChar;
      }
    }
  }
}


void setup() {
  Serial.begin(230400); // Inisialisasi komunikasi serial utama

  // --- LED GPS FIX (TAMBAHAN) ---
  pinMode(LED_GPS, OUTPUT);
  digitalWrite(LED_GPS, LOW); // Matikan LED saat start

  Serial.println("Mencoba koneksi ke GPS U-Blox...");
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); // Inisialisasi Serial 2 untuk GPS

  if (myGPS.begin(gpsSerial)) {
    Serial.println("Koneksi GPS berhasil!");
  } else {
    Serial.println("Gagal koneksi ke GPS. Cek kabel & baud rate.");
  }

  // Konfigurasi modul GPS U-Blox
  myGPS.setUART1Output(COM_TYPE_UBX); // Atur output ke format UBX (binary)
  myGPS.setNavigationFrequency(10); // Atur frekuensi update ke 10 Hz
  myGPS.setAutoPVT(true); // Aktifkan pengiriman data PVT (Position, Velocity, Time)

  uint8_t navFreq = myGPS.getNavigationFrequency();
  if (navFreq == 10) {
    Serial.println("GPS berhasil dikonfigurasi ke 10Hz.");
  } else {
    Serial.print("Konfigurasi 10Hz GAGAL! Frekuensi saat ini: ");
    Serial.print(navFreq);
    Serial.println(" Hz");
  }

  Wire.begin(21, 22); // Inisialisasi I2C (untuk Kompas CMPS12)

  // Inisialisasi Aktuator (Servo & ESC)
  rudderServo.attach(32);
  motorESC.attach(25);
  auxOutput.attach(23);

  // Posisi netral awal
  rudderServo.write(90);
  motorESC.writeMicroseconds(1500);
  auxOutput.writeMicroseconds(1500);

  // Inisialisasi PPM
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  loadDataFromMemory(); // Muat Waypoint yang tersimpan dari NVS

  Serial.println("");
  Serial.println("🌍 GPS + WAYPOINT SYSTEM (INTEGRATED)");
  Serial.println("================================");
  Serial.print("Data tersimpan: ");
  Serial.print(dataIndex);
  Serial.print("/");
  Serial.print(MAX_DATA);
  Serial.println(" titik");
  Serial.println("================================");
  Serial.print("Servo AI Inversi dimulai dari Waypoint ke-");
  Serial.println(AI_SERVO_INVERSION_INDEX + 1);
}

// --- Variabel Global Telemetri ---
float heading = 0.0;
double lat = 0.0, lon = 0.0;
double speed = 0.0; // km/jam
int sats = 0;
// ---------------------------------

void loop() {

  // --- 1. Baca Sensor GPS ---
  if (myGPS.getPVT()) { // Cek apakah ada data PVT baru dari GPS (10Hz)
    uint8_t fixType = myGPS.getFixType(); // Ambil status fix GPS

    // Nyalakan/Matikan LED berdasarkan status fix GPS (fixType > 0 berarti terkunci)
    if (fixType > 0) digitalWrite(LED_GPS, HIGH);
    else digitalWrite(LED_GPS, LOW);

    if (fixType > 0) { // Jika GPS sudah terkunci (2D atau 3D)
        lat = myGPS.getLatitude() / 10000000.0;
        lon = myGPS.getLongitude() / 10000000.0;
        speed = myGPS.getGroundSpeed() / 1000.0 * 3.6; // Kecepatan m/s dikonversi ke km/jam
        sats = myGPS.getSIV(); // Jumlah satelit yang terlihat
    } else { // Jika GPS belum terkunci
        lat = 0.0;
        lon = 0.0;
        speed = 0.0;
        sats = 0;
    }
  }
  
  // PANGGIL FUNGSI SERIAL NON-BLOCKING
  checkSerialInput(); // Membaca perintah dari Jetson tanpa menyebabkan delay
  
  // Baca Kompas
  heading = readCompass();
  if (heading == -1) { heading = 0.0; } // Jika gagal baca, anggap heading 0

  // Baca Channel Radio (PPM)
  int ch5 = readChannel(4); // Mode Switch (Manual/Auto)
  int ch6 = readChannel(5); // Waypoint Control (Record/Save/Clear)

  // Variabel Penampung Status & Output
  String mode = "MANUAL";
  String status = "ACTIVE";
  int finalServo = 90;
  int finalMotor = 1500;
  
  int wp_target_idx = 0;
  double wp_dist_m = 0.0;
  double wp_target_brg = 0.0;
  double wp_error_hdg = 0.0;

  // ----------------- MANUAL MODE -----------------
  if (ch5 < 1500) { // Jika CH5 < 1500 us (Manual)
    if (!isManual) {
      Serial.println("Switching to MANUAL...");
      auxOutput.writeMicroseconds(1500); // Aux netral
      isManual = true;
      wasInCaptureMode = false;
      wasInSaveMode = false;
    }

    mode = "MANUAL";

    int ch1 = readChannel(0); // Input Aileron/Rudder (Kemudi)
    int servoPosManual = map(ch1, 1000, 2000, 0, 180); // Mapping ke 0-180
    finalServo = servoPosManual;

    int ch3 = readChannel(2); // Input Throttle (Motor)
    finalMotor = ch3;

    int ch8 = readChannel(7); // Input Aux Output (misal: pompa)
    auxOutput.writeMicroseconds(ch8);

    // Logika Perekaman Waypoint (Hanya di mode Manual)
    if (ch6 >= 1400 && ch6 <= 1600) { // CH6 Posisi Tengah: Mode Rekam
      if (!wasInCaptureMode) {
        Serial.println("🟡 MODE REKAM: Siap merekam waypoint baru.");
        wasInCaptureMode = true;
        captureTriggered = false;
      }
    } else if (ch6 > 1900) { // CH6 Posisi Tinggi: Tekan & Simpan WP
      if (wasInCaptureMode && !captureTriggered) {
        // Logika Clear Data (jika sebelumnya sudah menekan tombol Save)
        if (wasInSaveMode) {
          clearAllData();
          wasInSaveMode = false;
        }
        // Simpan Waypoint baru
        if (dataIndex >= MAX_DATA) {
          Serial.println("⚠ Memori penuh. Tidak bisa menambah titik lagi.");
        } else {
          if (myGPS.getFixType() > 0) { // Cek lock GPS sebelum simpan
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
    } else if (ch6 < 1100) { // CH6 Posisi Rendah: Simpan semua data
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
  else { // Jika CH5 >= 1500 us (Auto)
    if (isManual) {
      Serial.println("Switching to AUTO...");
      isManual = false;
      counter = 0; // Reset target waypoint ke awal
    }

    mode = "AUTO";

    // PRIORITAS 1: Perintah dari AI (serialCommand == 'A')
    if (serialCommand == 'A') {
      int calculatedServoVal = ai_servo_val;
      
      // LOGIKA INVERSI SERVO AI BERDASARKAN WAYPOINT TARGET
      // Inversi dimulai ketika Waypoint target (counter) mencapai atau melebihi AI_SERVO_INVERSION_INDEX
      if (counter >= AI_SERVO_INVERSION_INDEX) {
        // Inversi: 180 -> 0, 175 -> 5, 90 -> 90, dst.
        calculatedServoVal = 180 - ai_servo_val; 
        
        // Batasan untuk memastikan nilai tetap dalam range 0-180
        if (calculatedServoVal > 180) calculatedServoVal = 180;
        if (calculatedServoVal < 0) calculatedServoVal = 0;
        
        status = "AI_INVERTED"; // Menambahkan status untuk indikasi inversi
        Serial.print("AI Servo Inverted (WP ");
        Serial.print(counter + 1);
        Serial.print("): ");
        Serial.print(ai_servo_val);
        Serial.print(" -> ");
        Serial.println(calculatedServoVal);
      } else {
        status = "AI_ACTIVE";
      }

      finalServo = calculatedServoVal;
      finalMotor = ai_motor_val;
      
    } 
    // PRIORITAS 2: Waypoint Navigation (serialCommand == 'W')
    else if (serialCommand == 'W') {
      status = "WAYPOINT";
      
      if (dataIndex > 0 && myGPS.getFixType() > 0) { // Cek ada data WP dan GPS fix
        
        // --- LOGIKA UTAMA WAYPOINT ---
        if (counter >= dataIndex) { // Semua WP selesai
          finalServo = 90;
          finalMotor = 1000; // Motor mati
          status = "WP_COMPLETE";
          wp_target_idx = dataIndex; // Mengirim jumlah total WP yang diselesaikan
        } else { // Navigasi ke WP berikutnya
          double targetLat = latitudes[counter];
          double targetLon = longitudes[counter];
          double dist = haversine(lat, lon, targetLat, targetLon); // Jarak ke WP
          double targetBearing = bearing(lat, lon, targetLat, targetLon); // Arah ke WP
          
          // Hitung Error Heading (untuk PID)
          double errorHeading = targetBearing - heading;
          if (errorHeading > 180) errorHeading -= 360;
          if (errorHeading < -180) errorHeading += 360;
          
          // Hitung output servo menggunakan PID
          int servoPos = PID_servo(targetBearing, heading);
          finalServo = servoPos;

          // Kontrol Motor: Menggunakan CH7 (Index 6) dari RC
          int motorSpeed = readChannel(6);  
          finalMotor = motorSpeed;

          // Cek apakah Waypoint sudah tercapai (radius 1.75m)
          if (dist < 1.75) {
            counter++; // Lanjut ke WP berikutnya
            // Tambahkan logging saat WP tercapai
            Serial.print("✅ WP #");
            Serial.print(counter);
            Serial.println(" tercapai. Menuju WP berikutnya.");
          }

          // Simpan data WP untuk telemetri JSON
          wp_target_idx = counter + 1; // Index WP yang sedang dituju (1-based)
          wp_dist_m = dist;
          wp_target_brg = targetBearing;
          wp_error_hdg = errorHeading;
        }
        // --- END LOGIKA UTAMA WAYPOINT ---
        
      } else {
        // Error handling jika tidak ada data atau GPS tidak fix
        finalServo = 90;
        finalMotor = 1000; // Stop motor saat error
        if (dataIndex == 0) status = "NO_WAYPOINTS";
        else status = "GPS_INVALID";
      }
    }
  }

  // --- 3. Kontrol Aktuator (Output PWM) ---
  rudderServo.write(finalServo); // Tulis nilai servo (0-180 deg)
  motorESC.writeMicroseconds(finalMotor); // Tulis nilai ESC (1000-2000 us)

  // ========================================
  // --- 4. BLOK TELEMETRI JSON ---
  // ========================================
  
  jsonDoc.clear(); // Bersihkan dokumen JSON sebelumnya

  jsonDoc["mode"] = mode;
  jsonDoc["status"] = status;

  // Data Sensor Inti 
  jsonDoc["heading"] = (float)round(heading * 100) / 100;
  jsonDoc["lat"] = lat;
  jsonDoc["lon"] = lon;
  jsonDoc["speed_kmh"] = (float)round(speed * 100) / 100;
  jsonDoc["sats"] = sats;

  // Data Output Aktuator
  jsonDoc["servo_out"] = finalServo;
  jsonDoc["motor_out"] = finalMotor;
  
  // Data Waypoint Inversi
  if (serialCommand == 'A') {
    jsonDoc["ai_inversion_active"] = (counter >= AI_SERVO_INVERSION_INDEX);
    jsonDoc["ai_wp_target"] = counter + 1;
    jsonDoc["ai_wp_start_invert"] = AI_SERVO_INVERSION_INDEX + 1;
  }

  // Data khusus Waypoint (HANYA DIKIRIM JIKA MODE AUTO DAN PERINTAH WAYPOINT)
  if (mode == "AUTO" && serialCommand == 'W') { 
    jsonDoc["wp_target_idx"] = wp_target_idx;
    
    // Khusus untuk status WP_COMPLETE, data lainnya (dist, brg, err) diatur 0 
    if (status == "WP_COMPLETE") {
      jsonDoc["wp_dist_m"] = 0.0;
      jsonDoc["wp_target_brg"] = 0.0;
      jsonDoc["wp_error_hdg"] = 0.0;
    } else {
      jsonDoc["wp_dist_m"] = (float)round(wp_dist_m * 100) / 100;
      jsonDoc["wp_target_brg"] = (float)round(wp_target_brg * 100) / 100;
      jsonDoc["wp_error_hdg"] = (float)round(wp_error_hdg * 100) / 100;
    }
  }

  // Kirim JSON ke Serial
  serializeJson(jsonDoc, Serial);
  Serial.println(); // Newline sebagai penanda akhir data

  // Jeda singkat agar loop tidak terlalu cepat (20Hz)
  delay(80); // Dipercepat dari 100ms menjadi 50ms untuk update 20Hz
}