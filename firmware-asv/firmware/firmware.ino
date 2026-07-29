#include <Wire.h> // Library untuk komunikasi I2C (untuk CMPS12)
#include <SparkFun_Ublox_Arduino_Library.h> // Library untuk GPS U-Blox
#include <ESP32Servo.h> // Library untuk mengontrol Servo dan ESC
#include <Preferences.h> // Library untuk menyimpan data di memori non-volatile (NVS/EEPROM)
#include <ArduinoJson.h> // Library untuk memproses dan mengirim data JSON (telemetri)

// ---------------- FREERTOS TASK HANDLES ----------------
TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskMotorHandle;

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
#define PIN_ESC_DEPAN_KIRI 27
#define PIN_ESC_DEPAN_KANAN 25
#define PIN_ESC_BAWAH_KIRI 26
#define PIN_ESC_BAWAH_KANAN 33

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

// --- Tuning Variables ---
volatile int servo_min_angle = 0;
volatile int servo_max_angle = 180;
volatile int thruster_manual_pwm = 1500;

// --- Waypoint Inversion Control for AI Mode ---
const int AI_SERVO_INVERSION_INDEX = 7; 

double error_val = 0, lastError = 0, integral = 0; // 'error' diganti agar aman

// ---------------- Waypoint ----------------
#define MAX_DATA 20 
Preferences preferences; 

float latitudes[MAX_DATA]; 
float longitudes[MAX_DATA]; 
volatile int dataIndex = 0; 
volatile int counter = 0; 

volatile bool captureTriggered = false; 
volatile bool wasInCaptureMode = false; 
volatile bool wasInSaveMode = false; 

// --- KONTROL DARI JETSON/KOMUNIKASI SERIAL ---
volatile char serialCommand = 'W'; 
volatile int ai_servo_val = 90; 
volatile int ai_dir_val = 1500; // Untuk direction (relays)
volatile int ai_motor_val = 1500; // Untuk motor bawah
volatile int ai_motor_depan_kiri_val = 1000;  // Nilai dari serial Jetson untuk motor depan kiri
volatile int ai_motor_depan_kanan_val = 1000; // Nilai dari serial Jetson untuk motor depan kanan

// --- Buffer JSON & Serial ---
StaticJsonDocument<400> jsonDoc;
String serialInputBuffer = ""; 

// --- Variabel Global Telemetri (Volatile untuk IPC) ---
volatile float heading = 0.0;
volatile double lat = 0.0, lon = 0.0;
volatile double speed = 0.0; 
volatile int sats = 0;

volatile bool isManual = true;
volatile int last_rc_ch5 = 1000; 

// Variabel penampung state untuk telemetri
volatile int finalServo_g = 90;
volatile int finalMotor_g = 1500;           
volatile int finalMotorDepanKiri_g = 1000;  
volatile int finalMotorDepanKanan_g = 1000; 
volatile int finalDir_g = 1500;             

volatile int wp_target_idx_g = 0;
volatile double wp_dist_m_g = 0.0;
volatile double wp_target_brg_g = 0.0;
volatile double wp_error_hdg_g = 0.0;

char mode_g[16] = "MANUAL";
char status_g[16] = "ACTIVE";
// -----------------------------------------------------

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
  error_val = input - setpoint; 

  if (error_val > 180) error_val -= 360; 
  if (error_val < -180) error_val += 360;

  integral += error_val; 
  double derivative = error_val - lastError; 
  lastError = error_val; 

  double output = Kp * error_val + Ki * integral + Kd * derivative; 
  int servoPos = 90 + output; 

  if (servoPos > servo_max_angle) servoPos = servo_max_angle;
  if (servoPos < servo_min_angle) servoPos = servo_min_angle;

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

// --- FUNGSI MEMBACA PERINTAH SERIAL ---
void checkSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read(); 
    
    if (incomingChar == '\n') {
      serialInputBuffer.trim(); 
      
      if (serialInputBuffer.length() > 0) {
        serialCommand = serialInputBuffer.charAt(0); 
        
        if (serialCommand == 'A') {
          int comma1 = serialInputBuffer.indexOf(',');
          int comma2 = serialInputBuffer.indexOf(',', comma1 + 1);
          int comma3 = serialInputBuffer.indexOf(',', comma2 + 1);
          int comma4 = serialInputBuffer.indexOf(',', comma3 + 1);
          int comma5 = serialInputBuffer.indexOf(',', comma4 + 1);

          if (comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0 && comma5 > 0) {
            String servoStr          = serialInputBuffer.substring(comma1 + 1, comma2);
            String dirStr            = serialInputBuffer.substring(comma2 + 1, comma3);
            String motorBwhStr       = serialInputBuffer.substring(comma3 + 1, comma4);
            String motorDepanKiriStr = serialInputBuffer.substring(comma4 + 1, comma5);
            String motorDepanKananStr= serialInputBuffer.substring(comma5 + 1);

            ai_servo_val             = servoStr.toInt();
            ai_dir_val               = dirStr.toInt();
            ai_motor_val             = motorBwhStr.toInt();
            ai_motor_depan_kiri_val  = motorDepanKiriStr.toInt();
            ai_motor_depan_kanan_val = motorDepanKananStr.toInt();
          }
        }
        else if (serialCommand == 'M') {
          int comma1 = serialInputBuffer.indexOf(',');
          if (comma1 > 0) {
            String modeStr = serialInputBuffer.substring(comma1 + 1);
            modeStr.trim();
            if (modeStr == "AUTO" && isManual) {
              Serial.println("[Serial CMD] Switching to AUTO (from GUI)...");
              isManual = false;
              counter = 0;
            } else if (modeStr == "MANUAL" && !isManual) {
              Serial.println("[Serial CMD] Switching to MANUAL (from GUI)...");
              isManual = true;
              wasInCaptureMode = false;
              wasInSaveMode = false;
            }
          }
        }
        else if (serialCommand == 'T') {
          int comma1 = serialInputBuffer.indexOf(',');
          int comma2 = serialInputBuffer.indexOf(',', comma1 + 1);
          int comma3 = serialInputBuffer.indexOf(',', comma2 + 1);
          int comma4 = serialInputBuffer.indexOf(',', comma3 + 1);
          
          if (comma1 > 0 && comma2 > 0) {
            String typeStr = serialInputBuffer.substring(comma1 + 1, comma2);
            if (typeStr == "PID" && comma3 > 0 && comma4 > 0) {
              String pStr = serialInputBuffer.substring(comma2 + 1, comma3);
              String iStr = serialInputBuffer.substring(comma3 + 1, comma4);
              String dStr = serialInputBuffer.substring(comma4 + 1);
              
              Kp = pStr.toDouble();
              Ki = iStr.toDouble();
              Kd = dStr.toDouble();
              
              Serial.println("ACK:PID_UPDATED");
            }
            else if (typeStr == "SRV" && comma3 > 0) {
              String leftStr = serialInputBuffer.substring(comma2 + 1, comma3);
              String rightStr = serialInputBuffer.substring(comma3 + 1);
              
              servo_min_angle = leftStr.toInt();
              servo_max_angle = rightStr.toInt();
              
              Serial.println("ACK:SERVO_UPDATED");
            }
            else if (typeStr == "THR") {
              String speedStr = (comma3 > 0) ? serialInputBuffer.substring(comma2 + 1, comma3) : serialInputBuffer.substring(comma2 + 1);
              int pct = speedStr.toInt();
              thruster_manual_pwm = 1500 + (pct * 5); // 0-100% -> 1500-2000 PWM
              
              Serial.println("ACK:THRUSTER_UPDATED");
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


// ==========================================================
// FREERTOS TASKS
// ==========================================================

void TaskSensor(void *pvParameters) {
  for(;;) {
    // 1. Baca GPS
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

    // 2. Baca Kompas
    float temp_heading = readCompass();
    if (temp_heading != -1) {
      heading = temp_heading;
    }

    // 3. Telemetri JSON
    jsonDoc.clear(); 

    jsonDoc["mode"] = String(mode_g);
    jsonDoc["status"] = String(status_g);

    jsonDoc["heading"] = (float)round(heading * 100) / 100;
    jsonDoc["lat"] = lat;
    jsonDoc["lon"] = lon;
    jsonDoc["speed_kmh"] = (float)round(speed * 100) / 100;
    jsonDoc["sats"] = sats;

    jsonDoc["servo_out"] = finalServo_g;
    jsonDoc["motor_bwh_out"] = finalMotor_g;
    jsonDoc["motor_d_kiri_out"] = finalMotorDepanKiri_g;
    jsonDoc["motor_d_kanan_out"] = finalMotorDepanKanan_g;
    
    if (serialCommand == 'A') {
      jsonDoc["ai_inversion_active"] = (counter >= AI_SERVO_INVERSION_INDEX);
      jsonDoc["ai_wp_target"] = counter + 1;
      jsonDoc["ai_wp_start_invert"] = AI_SERVO_INVERSION_INDEX + 1;
    }

    if (String(mode_g) == "AUTO" && serialCommand == 'W') { 
      jsonDoc["wp_target_idx"] = wp_target_idx_g;
      
      if (String(status_g) == "WP_COMPLETE") {
        jsonDoc["wp_dist_m"] = 0.0;
        jsonDoc["wp_target_brg"] = 0.0;
        jsonDoc["wp_error_hdg"] = 0.0;
      } else {
        jsonDoc["wp_dist_m"] = (float)round(wp_dist_m_g * 100) / 100;
        jsonDoc["wp_target_brg"] = (float)round(wp_target_brg_g * 100) / 100;
        jsonDoc["wp_error_hdg"] = (float)round(wp_error_hdg_g * 100) / 100;
      }
    }

    serializeJson(jsonDoc, Serial);
    Serial.println(); 

    // Jalankan task ini ~10Hz
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskMotor(void *pvParameters) {
  for(;;) {
    checkSerialInput(); 

    int ch5 = readChannel(4); 
    int ch6 = readChannel(5); 

    String local_mode = "MANUAL";
    String local_status = "ACTIVE";
    
    int finalServo = 90;
    int finalMotor = 1500;           
    int finalMotorDepanKiri = 1000;  
    int finalMotorDepanKanan = 1000; 
    int finalDir = 1500;             
    
    int wp_target_idx = 0;
    double wp_dist_m = 0.0;
    double wp_target_brg = 0.0;
    double wp_error_hdg = 0.0;

    bool rc_wants_manual = (ch5 < 1500);
    bool rc_was_manual = (last_rc_ch5 < 1500);
    
    if (rc_wants_manual != rc_was_manual) {
      if (rc_wants_manual && !isManual) {
        Serial.println("[RC Edge] Switching to MANUAL...");
        isManual = true;
        wasInCaptureMode = false;
        wasInSaveMode = false;
      } else if (!rc_wants_manual && isManual) {
        Serial.println("[RC Edge] Switching to AUTO...");
        isManual = false;
        counter = 0;
      }
    }
    last_rc_ch5 = ch5; 

    // ----------------- MANUAL MODE -----------------
    if (isManual) {
      local_mode = "MANUAL";

      int ch1 = readChannel(0); 
      finalServo = map(ch1, 1000, 2000, servo_min_angle, servo_max_angle); 

      int ch3 = readChannel(2); 
      finalMotor = ch3;

      finalMotorDepanKiri = 1000;
      finalMotorDepanKanan = 1000;

      int ch8 = readChannel(7);
      finalDir = ch8;

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
            if (lat != 0.0 && lon != 0.0) { 
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
      local_mode = "AUTO";
      finalDir = ai_dir_val; 

      if (serialCommand == 'A') {
        int calculatedServoVal = ai_servo_val;
        
        if (counter >= AI_SERVO_INVERSION_INDEX) {
          calculatedServoVal = 180 - ai_servo_val; 
          
          if (calculatedServoVal > servo_max_angle) calculatedServoVal = servo_max_angle;
          if (calculatedServoVal < servo_min_angle) calculatedServoVal = servo_min_angle;
          
          local_status = "AI_INVERTED"; 
        } else {
          local_status = "AI_ACTIVE";
        }

        finalServo           = calculatedServoVal;
        finalMotor           = ai_motor_val; 
        finalMotorDepanKiri  = ai_motor_depan_kiri_val;
        finalMotorDepanKanan = ai_motor_depan_kanan_val;
        
      } 
      else if (serialCommand == 'W') {
        local_status = "WAYPOINT";
        finalDir = 2000; // Maju
        
        if (dataIndex > 0 && lat != 0.0 && lon != 0.0) { 
          
          if (counter >= dataIndex) { 
            finalServo = 90;
            finalMotor = 1000; 
            finalMotorDepanKiri = 1000;  
            finalMotorDepanKanan = 1000; 
            local_status = "WP_COMPLETE";
            wp_target_idx = dataIndex; 
          } else { 
            double targetLat = latitudes[counter];
            double targetLon = longitudes[counter];
            double dist = haversine(lat, lon, targetLat, targetLon); 
            double targetBearing = bearing(lat, lon, targetLat, targetLon); 
            
            double errorHeading = targetBearing - heading;
            if (errorHeading > 180) errorHeading -= 360;
            if (errorHeading < -180) errorHeading += 360;
            
            int servoPos = PID_servo(targetBearing, heading);
            finalServo = servoPos;

            int motorSpeed = readChannel(6);  
            finalMotor = motorSpeed;          
            
            finalMotorDepanKiri = 1000;       
            finalMotorDepanKanan = 1000;      

            if (dist < 1.75) {
              counter++; 
              Serial.print("✅ WP #");
              Serial.print(counter);
              Serial.println(" tercapai. Menuju WP berikutnya.");
            }

            wp_target_idx = counter + 1; 
            wp_dist_m = dist;
            wp_target_brg = targetBearing;
            wp_error_hdg = errorHeading;
          }
          
        } else {
          finalServo = 90;
          finalMotor = 1000; 
          finalMotorDepanKiri = 1000;   
          finalMotorDepanKanan = 1000;  
          if (dataIndex == 0) local_status = "NO_WAYPOINTS";
          else local_status = "GPS_INVALID";
        }
      }
    }

    // Eksekusi Hardware
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

    // Update global telemetry variables
    local_mode.toCharArray(mode_g, 16);
    local_status.toCharArray(status_g, 16);
    finalServo_g = finalServo;
    finalMotor_g = finalMotor;
    finalMotorDepanKiri_g = finalMotorDepanKiri;
    finalMotorDepanKanan_g = finalMotorDepanKanan;
    finalDir_g = finalDir;
    
    wp_target_idx_g = wp_target_idx;
    wp_dist_m_g = wp_dist_m;
    wp_target_brg_g = wp_target_brg;
    wp_error_hdg_g = wp_error_hdg;

    // Jalankan task ini ~50Hz
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==========================================================
// SETUP & LOOP
// ==========================================================

void setup() {
  Serial.begin(230400); 

  Serial.println("Menginisialisasi ESC, Pin Arah, dan Servo...");
  escDepanKiri.attach(PIN_ESC_DEPAN_KIRI);
  escDepanKanan.attach(PIN_ESC_DEPAN_KANAN);
  escBawahKiri.attach(PIN_ESC_BAWAH_KIRI);
  escBawahKanan.attach(PIN_ESC_BAWAH_KANAN);

  dirDepanKiri.attach(DIR_DEPAN_KIRI);
  dirDepanKanan.attach(DIR_DEPAN_KANAN);
  dirBawahKiri.attach(DIR_BAWAH_KIRI);
  dirBawahKanan.attach(DIR_BAWAH_KANAN);

  servoKiri.attach(PIN_SERVO_KIRI);
  servoKanan.attach(PIN_SERVO_KANAN);

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

  // INISIALISASI FREERTOS TASKS
  xTaskCreatePinnedToCore(
    TaskSensor,        // Fungsi Task
    "TaskSensor",      // Nama Task
    4096,              // Ukuran Stack
    NULL,              // Parameter (opsional)
    1,                 // Prioritas (1 = sedang)
    &TaskSensorHandle, // Task Handle
    0                  // Pinned to Core 0
  );

  xTaskCreatePinnedToCore(
    TaskMotor,         // Fungsi Task
    "TaskMotor",       // Nama Task
    4096,              // Ukuran Stack
    NULL,              // Parameter
    2,                 // Prioritas (2 = tinggi, Motor harus real-time)
    &TaskMotorHandle,  // Task Handle
    1                  // Pinned to Core 1
  );
}

void loop() {
  // Loop bawaan Arduino di Core 1 dikosongkan.
  // Semua tugas sudah ditangani oleh FreeRTOS.
  vTaskDelete(NULL);
}