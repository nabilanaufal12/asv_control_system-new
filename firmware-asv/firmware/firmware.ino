#include <Wire.h> // Library untuk komunikasi I2C (untuk CMPS12)
#include <SparkFun_Ublox_Arduino_Library.h> // Library untuk GPS U-Blox
#include <ESP32Servo.h> // Library untuk mengontrol Servo dan ESC
#include <Preferences.h> // Library untuk menyimpan data di memori non-volatile (NVS/EEPROM)
#include <ArduinoJson.h> // Library untuk memproses dan mengirim data JSON (telemetri)

// ---------------- FREERTOS TASK HANDLES & MUTEX ----------------
TaskHandle_t TaskSensorHandle;
TaskHandle_t TaskMotorHandle;
SemaphoreHandle_t stateMutex; // Mutex untuk proteksi data antar core (IPC)

// ---------------- GPS (U-Blox 10Hz) ----------------
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
#define PIN_SERVO_KIRI 32
#define PIN_SERVO_KANAN 23

#define PIN_ESC_DEPAN_KIRI 27
#define PIN_ESC_DEPAN_KANAN 25
#define PIN_ESC_BAWAH_KIRI 26
#define PIN_ESC_BAWAH_KANAN 33

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
volatile int servo_min_angle = 0;
volatile int servo_max_angle = 180;
volatile int thruster_manual_pwm = 1500;

const int AI_SERVO_INVERSION_INDEX = 7; 
double error_val = 0, lastError = 0, integral = 0; // Hanya diakses oleh TaskSensor, tidak perlu Mutex

// ---------------- Waypoint ----------------
#define MAX_DATA 20 
Preferences preferences; 

float latitudes[MAX_DATA]; 
float longitudes[MAX_DATA]; 
int dataIndex = 0; 
int counter = 0; 

// --- Variabel flag aksi (Protected by Mutex) ---
bool captureTriggered = false; 
bool saveTriggered = false;
bool clearTriggered = false;

// --- KONTROL DARI JETSON/KOMUNIKASI SERIAL (Protected by Mutex) ---
char serialCommand = 'W'; 
int ai_servo_val = 90; 
int ai_dir_val = 1500; 
int ai_motor_val = 1500; 
int ai_dir_depan_kiri_val = 1500;
int ai_motor_depan_kiri_val = 1000;  
int ai_dir_depan_kanan_val = 1500;
int ai_motor_depan_kanan_val = 1000; 

// --- Buffer JSON & Serial ---
StaticJsonDocument<400> jsonDoc;
char serialInputBuffer[128];
int serialInputIndex = 0;

// --- Variabel Global Telemetri (Protected by Mutex) ---
float heading = 0.0;
double lat = 0.0, lon = 0.0;
double speed = 0.0; 
int sats = 0;

bool isManual = true;
int last_rc_ch5 = 1000; 

// Output final aktuator
int finalServo_g = 90;
int finalMotor_g = 1500;           
int finalMotorDepanKiri_g = 1000;  
int finalMotorDepanKanan_g = 1000; 
int finalDir_g = 1500;             
int finalDirDepanKiri_g = 1500;
int finalDirDepanKanan_g = 1500;

int wp_target_idx_g = 0;
double wp_dist_m_g = 0.0;
double wp_target_brg_g = 0.0;
double wp_error_hdg_g = 0.0;

char mode_g[16] = "MANUAL";
char status_g[16] = "ACTIVE";

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
    char latKey[10];
    char lngKey[10];
    sprintf(latKey, "lat%d", i);
    sprintf(lngKey, "lng%d", i);
    preferences.putFloat(latKey, latitudes[i]);
    preferences.putFloat(lngKey, longitudes[i]);
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
    char latKey[10];
    char lngKey[10];
    sprintf(latKey, "lat%d", i);
    sprintf(lngKey, "lng%d", i);
    latitudes[i] = preferences.getFloat(latKey, 0.0);
    longitudes[i] = preferences.getFloat(lngKey, 0.0);
  }
  preferences.end(); 
}

void clearAllData() {
  preferences.begin("gps-data", false);
  preferences.clear();
  preferences.end();
  dataIndex = 0;
  // Serial.println() dihindari karena dipanggil dari TaskMotor
}

// --- FUNGSI MEMBACA PERINTAH SERIAL (Dipanggil oleh TaskMotor) ---
void checkSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read(); 
    
    if (incomingChar == '\n') {
      serialInputBuffer[serialInputIndex] = '\0'; // Null-terminate
      
      if (serialInputIndex > 0) {
        char cmd = serialInputBuffer[0];
        
        if (cmd == 'A') {
          // Format Baru: A,servo,dir,motorBwh,dirDepanKiri,motorDepanKiri,dirDepanKanan,motorDepanKanan
          int c1, c2, c3, c4, c5, c6, c7;
          if (sscanf(serialInputBuffer, "A,%d,%d,%d,%d,%d,%d,%d", &c1, &c2, &c3, &c4, &c5, &c6, &c7) == 7) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            serialCommand = cmd;
            ai_servo_val = c1;
            ai_dir_val = c2;
            ai_motor_val = c3;
            ai_dir_depan_kiri_val = c4;
            ai_motor_depan_kiri_val = c5;
            ai_dir_depan_kanan_val = c6;
            ai_motor_depan_kanan_val = c7;
            xSemaphoreGive(stateMutex);
          }
        }
        else if (cmd == 'W') {
           xSemaphoreTake(stateMutex, portMAX_DELAY);
           serialCommand = cmd;
           xSemaphoreGive(stateMutex);
        }
        else if (cmd == 'M') {
          if (strncmp(serialInputBuffer, "M,AUTO", 6) == 0) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            if (isManual) {
              isManual = false;
              counter = 0;
            }
            xSemaphoreGive(stateMutex);
          } else if (strncmp(serialInputBuffer, "M,MANUAL", 8) == 0) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            if (!isManual) {
              isManual = true;
            }
            xSemaphoreGive(stateMutex);
          }
        }
        else if (cmd == 'T') {
          double p, i, d;
          int minA, maxA, pct;
          if (sscanf(serialInputBuffer, "T,PID,%lf,%lf,%lf", &p, &i, &d) == 3) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            Kp = p; Ki = i; Kd = d;
            xSemaphoreGive(stateMutex);
          }
          else if (sscanf(serialInputBuffer, "T,SRV,%d,%d", &minA, &maxA) == 2) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            servo_min_angle = minA;
            servo_max_angle = maxA;
            xSemaphoreGive(stateMutex);
          }
          else if (sscanf(serialInputBuffer, "T,THR,%d", &pct) == 1) {
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            thruster_manual_pwm = 1500 + (pct * 5); // 0-100% -> 1500-2000 PWM
            xSemaphoreGive(stateMutex);
          }
        }
      }
      serialInputIndex = 0;
    } else if (incomingChar != '\r') {
      if (serialInputIndex < sizeof(serialInputBuffer) - 1) { 
        serialInputBuffer[serialInputIndex++] = incomingChar;
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
    bool gotGPS = myGPS.getPVT();
    uint8_t fixType = gotGPS ? myGPS.getFixType() : 0;
    
    // 2. Baca Kompas
    float temp_heading = readCompass();

    // -- AMBIL MUTEX UNTUK MEMBACA & MEMPERBARUI STATE --
    xSemaphoreTake(stateMutex, portMAX_DELAY);

    // Update LED dan Variabel GPS global
    digitalWrite(LED_GPS, fixType > 0 ? HIGH : LOW);
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

    if (temp_heading != -1) {
      heading = temp_heading;
    }

    // ----------------------------------------------------
    // NAVIGASI (HANYA DIEKSEKUSI DI LOOP SENSOR ~10Hz)
    // ----------------------------------------------------
    int finalServo = 90;
    int finalMotor = 1500;
    int finalMotorDepanKiri = 1000;
    int finalMotorDepanKanan = 1000;
    int finalDir = 1500;
    int finalDirDepanKiri = 1500;
    int finalDirDepanKanan = 1500;
    
    char local_mode[16];
    char local_status[16];
    strcpy(local_mode, isManual ? "MANUAL" : "AUTO");
    strcpy(local_status, "ACTIVE");

    int wp_target_idx = 0;
    double wp_dist_m = 0.0;
    double wp_target_brg = 0.0;
    double wp_error_hdg = 0.0;

    if (!isManual) {
       finalDir = ai_dir_val;
       finalDirDepanKiri = ai_dir_depan_kiri_val;
       finalDirDepanKanan = ai_dir_depan_kanan_val;

       if (serialCommand == 'A') {
         int calculatedServoVal = ai_servo_val;
         
         if (counter >= AI_SERVO_INVERSION_INDEX) {
           calculatedServoVal = 180 - ai_servo_val; 
           if (calculatedServoVal > servo_max_angle) calculatedServoVal = servo_max_angle;
           if (calculatedServoVal < servo_min_angle) calculatedServoVal = servo_min_angle;
           strcpy(local_status, "AI_INVERTED");
         } else {
           strcpy(local_status, "AI_ACTIVE");
         }
         
         finalServo           = calculatedServoVal;
         finalMotor           = ai_motor_val; 
         finalMotorDepanKiri  = ai_motor_depan_kiri_val;
         finalMotorDepanKanan = ai_motor_depan_kanan_val;
       } 
       else if (serialCommand == 'W') {
         strcpy(local_status, "WAYPOINT");
         finalDir = 2000;
         finalDirDepanKiri = 2000;
         finalDirDepanKanan = 2000;
         
         if (dataIndex > 0 && lat != 0.0 && lon != 0.0) { 
           if (counter >= dataIndex) { 
             finalServo = 90;
             finalMotor = 1000; 
             finalMotorDepanKiri = 1000;  
             finalMotorDepanKanan = 1000; 
             strcpy(local_status, "WP_COMPLETE");
             wp_target_idx = dataIndex; 
           } else { 
             double targetLat = latitudes[counter];
             double targetLon = longitudes[counter];
             double dist = haversine(lat, lon, targetLat, targetLon); 
             double targetBearing = bearing(lat, lon, targetLat, targetLon); 
             
             double errorHeading = targetBearing - heading;
             if (errorHeading > 180) errorHeading -= 360;
             if (errorHeading < -180) errorHeading += 360;
             
             // Hitung PID di 10Hz
             finalServo = PID_servo(targetBearing, heading);
             
             // Ambil nilai Motor dari TaskMotor/PPM (Disinkronkan)
             finalMotor = readChannel(6); 
             finalMotorDepanKiri = 1000;       
             finalMotorDepanKanan = 1000;      

             if (dist < 1.75) {
               counter++; // Capai WP
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
           strcpy(local_status, dataIndex == 0 ? "NO_WAYPOINTS" : "GPS_INVALID");
         }
       }
       
       // Terapkan hasil perhitungan ke Global Variable agar dieksekusi TaskMotor
       finalServo_g = finalServo;
       finalMotor_g = finalMotor;
       finalMotorDepanKiri_g = finalMotorDepanKiri;
       finalMotorDepanKanan_g = finalMotorDepanKanan;
       finalDir_g = finalDir;
       finalDirDepanKiri_g = finalDirDepanKiri;
       finalDirDepanKanan_g = finalDirDepanKanan;
       
       wp_target_idx_g = wp_target_idx;
       wp_dist_m_g = wp_dist_m;
       wp_target_brg_g = wp_target_brg;
       wp_error_hdg_g = wp_error_hdg;
       
       strcpy(mode_g, local_mode);
       strcpy(status_g, local_status);
    } 
    
    // --- Manajemen Simpan Waypoint (Dipindahkan ke Sensor Loop) ---
    if (isManual) {
      if (clearTriggered) {
         clearAllData();
         clearTriggered = false;
      }
      if (captureTriggered) {
         if (dataIndex < MAX_DATA && lat != 0.0 && lon != 0.0) {
            latitudes[dataIndex] = lat;
            longitudes[dataIndex] = lon;
            dataIndex++;
            saveDataToMemory();
         }
         captureTriggered = false;
      }
      if (saveTriggered) {
         saveDataToMemory();
         saveTriggered = false;
      }
    }

    // --- TELEMETRI JSON (HANYA TASK SENSOR YANG BOLEH SERIAL PRINT) ---
    jsonDoc.clear(); 

    jsonDoc["mode"] = mode_g;
    jsonDoc["status"] = status_g;

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

    if (strcmp(mode_g, "AUTO") == 0 && serialCommand == 'W') { 
      jsonDoc["wp_target_idx"] = wp_target_idx_g;
      
      if (strcmp(status_g, "WP_COMPLETE") == 0) {
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

    // LEPASKAN MUTEX
    xSemaphoreGive(stateMutex);

    // Jalankan task ini ~10Hz
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskMotor(void *pvParameters) {
  bool local_wasInCaptureMode = false;
  bool local_wasInSaveMode = false;

  for(;;) {
    // 1. Baca Serial (Menggunakan Char array, NO STRING HEAP!)
    checkSerialInput(); 

    // 2. Baca RC Input
    int ch1 = readChannel(0); 
    int ch3 = readChannel(2); 
    int ch5 = readChannel(4); 
    int ch6 = readChannel(5); 
    int ch8 = readChannel(7);

    // -- AMBIL MUTEX UNTUK MEMBACA/MENULIS STATE --
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    
    bool rc_wants_manual = (ch5 < 1500);
    bool rc_was_manual = (last_rc_ch5 < 1500);
    
    if (rc_wants_manual != rc_was_manual) {
      if (rc_wants_manual && !isManual) {
        isManual = true;
        local_wasInCaptureMode = false;
        local_wasInSaveMode = false;
      } else if (!rc_wants_manual && isManual) {
        isManual = false;
        counter = 0;
      }
    }
    last_rc_ch5 = ch5; 

    // ----------------- MANUAL MODE -----------------
    if (isManual) {
      strcpy(mode_g, "MANUAL");
      strcpy(status_g, "ACTIVE");

      finalServo_g = map(ch1, 1000, 2000, servo_min_angle, servo_max_angle); 
      finalMotor_g = ch3;
      finalMotorDepanKiri_g = 1000;
      finalMotorDepanKanan_g = 1000;
      finalDir_g = ch8;
      finalDirDepanKiri_g = ch8;
      finalDirDepanKanan_g = ch8;

      // Handle RC switch mode
      if (ch6 >= 1400 && ch6 <= 1600) { 
        if (!local_wasInCaptureMode) {
          local_wasInCaptureMode = true;
        }
      } else if (ch6 > 1900) { 
        if (local_wasInCaptureMode && !captureTriggered) {
          if (local_wasInSaveMode) {
            clearTriggered = true; // Signal sensor task to clear
            local_wasInSaveMode = false;
          }
          captureTriggered = true; // Signal sensor task to save point
        }
        local_wasInCaptureMode = false;
      } else if (ch6 < 1100) { 
        if (!local_wasInSaveMode) {
          saveTriggered = true; // Signal sensor task to write preferences
          local_wasInSaveMode = true;
        }
        local_wasInCaptureMode = false;
      }
    }

    // Ambil variabel ke stack lokal agar aman saat eksekusi IO hardware
    int srv = finalServo_g;
    int mtr = finalMotor_g;
    int mdk = finalMotorDepanKiri_g;
    int mdk_r = finalMotorDepanKanan_g;
    int dir = finalDir_g;
    int ddk = finalDirDepanKiri_g;
    int ddk_r = finalDirDepanKanan_g;
    
    // LEPASKAN MUTEX
    xSemaphoreGive(stateMutex);

    // 3. Eksekusi Hardware
    servoKiri.write(srv); 
    servoKanan.write(srv); 

    dirDepanKiri.writeMicroseconds(ddk);
    dirDepanKanan.writeMicroseconds(ddk_r);
    dirBawahKiri.writeMicroseconds(dir);
    dirBawahKanan.writeMicroseconds(dir);

    escDepanKiri.writeMicroseconds(mdk);
    escDepanKanan.writeMicroseconds(mdk_r);
    escBawahKiri.writeMicroseconds(mtr);
    escBawahKanan.writeMicroseconds(mtr);

    // Jalankan task ini pada ~50Hz untuk kelancaran Motor dan RC
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==========================================================
// SETUP & LOOP
// ==========================================================

void setup() {
  Serial.begin(230400); 

  // Inisialisasi Mutex
  stateMutex = xSemaphoreCreateMutex();

  // Inisialisasi Aktuator
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

  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 
  myGPS.begin(gpsSerial);
  myGPS.setUART1Output(COM_TYPE_UBX); 
  myGPS.setNavigationFrequency(10); 
  myGPS.setAutoPVT(true); 

  Wire.begin(21, 22); 

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  loadDataFromMemory(); 

  // INISIALISASI FREERTOS TASKS
  xTaskCreatePinnedToCore(
    TaskSensor,        // Fungsi Task
    "TaskSensor",      // Nama Task
    4096,              // Ukuran Stack
    NULL,              // Parameter
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
  // Semua tugas sudah ditangani oleh FreeRTOS.
  vTaskDelete(NULL);
}