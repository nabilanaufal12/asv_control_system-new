#include <Wire.h> 
#include <SparkFun_Ublox_Arduino_Library.h> 
#include <ESP32Servo.h> 
#include <Preferences.h> 
#include <ArduinoJson.h> 

// ---------------- GPS (U-Blox 10Hz) ----------------
SFE_UBLOX_GPS myGPS;
HardwareSerial gpsSerial(2); 
#define GPS_BAUD_RATE 9600 
#define GPS_RX_PIN 16 
#define GPS_TX_PIN 17 
#define LED_GPS 2 

// ---------------- CMPS12 ----------------
#define CMPS12_ADDRESS 0x60 
#define ANGLE_16BIT_REGISTER 2 

// ---------------- Konfigurasi Pin Aktuator ----------------
#define PIN_SERVO_KIRI 32
#define PIN_SERVO_KANAN 23

// PIN ESC TERBARU
#define PIN_ESC_DEPAN_KIRI 26
#define PIN_ESC_DEPAN_KANAN 33
#define PIN_ESC_BAWAH_KIRI 27
#define PIN_ESC_BAWAH_KANAN 25

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
double Kp = 2.0, Ki = 0.0, Kd = 0.5; 
const int AI_SERVO_INVERSION_INDEX = 7; 
double error_val, lastError = 0, integral = 0;

// ---------------- Waypoint ----------------
#define MAX_DATA 20 
Preferences preferences; 

float latitudes[MAX_DATA]; 
float longitudes[MAX_DATA]; 
int dataIndex = 0; 
int counter = 0; 

bool captureTriggered = false; 
bool wasInCaptureMode = false; 
bool wasInSaveMode = false; 

// --- [BARU] Parameter LOS (Line of Sight) ---
double L_delta = 3.0;           // Lookahead distance dalam meter. (Bisa di-tuning, misal 2.0 - 5.0)
double prev_wp_lat = 0.0;       // Latitude awal lintasan (Waypoint sebelumnya)
double prev_wp_lon = 0.0;       // Longitude awal lintasan
bool is_new_wp = true;          // Penanda untuk membuat garis lintasan baru
double cross_track_error = 0.0; // Nilai simpangan kapal dari jalur lurus

// --- [BARU] State Machine Misi Khusus ---
enum MissionState {
  STATE_NORMAL_NAV,
  STATE_PHOTO_LOITER,
  STATE_PHOTO_REVERSE,
  STATE_DOCKING
};
MissionState currentMissionState = STATE_NORMAL_NAV;
unsigned long missionStateStartTime = 0;

// --- KONTROL DARI JETSON ---
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

// --- Variabel Global Telemetri ---
float heading = 0.0;
double lat = 0.0, lon = 0.0;
double speed = 0.0; 
int sats = 0;
bool isManual = true;

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
  if (servoPos > 180) servoPos = 180;
  if (servoPos < 0) servoPos = 0;
  return servoPos; 
}

// ---------------- PPM INPUT ----------------
#define PPM_PIN 4 
#define CHANNELS 10 
volatile int ppm_temp[CHANNELS]; 
volatile int ppm_valid[CHANNELS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
volatile byte ppmCounter = 0; 
volatile unsigned long lastMicros = 0; 

void IRAM_ATTR ppmISR() {
  unsigned long now = micros();
  unsigned long diff = now - lastMicros;
  lastMicros = now;

  if (diff > 5000) { 
    if (ppmCounter >= 6) { 
      for(int i=0; i<CHANNELS; i++) {
        ppm_valid[i] = ppm_temp[i];
      }
    }
    ppmCounter = 0;
  } else {
    if (ppmCounter < CHANNELS) {
      ppm_temp[ppmCounter] = diff; 
      ppmCounter++;
    }
  }
}

bool rc_initialized = false;
bool throttle_armed = false;
int valid_frame_count = 0;

int readChannel(byte ch, int minVal = 1000, int maxVal = 2000, int defaultVal = 1500) {
  static float smoothedValid[CHANNELS] = {1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0};
  const float alpha = 0.4; 

  if (ch < CHANNELS) {
    int val = ppm_valid[ch];
    if (val >= 900 && val <= 2100) {
      if (!rc_initialized) {
        smoothedValid[ch] = val; 
        return val;
      }
      smoothedValid[ch] = (alpha * val) + ((1.0 - alpha) * smoothedValid[ch]);
      return (int)smoothedValid[ch];
    }
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
}

// --- FUNGSI MEMBACA PERINTAH SERIAL ---
void checkSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read(); 
    if (incomingChar == '\n') {
      serialInputBuffer[serialInputIndex] = '\0';
      if (serialInputIndex > 0) {
        char cmd = serialInputBuffer[0];
        if (cmd == 'A') {
          int srv, r_dir, r_pwm, fl_dir, fl_pwm, fr_dir, fr_pwm;
          if (sscanf(serialInputBuffer, "A,%d,%d,%d,%d,%d,%d,%d", &srv, &r_dir, &r_pwm, &fl_dir, &fl_pwm, &fr_dir, &fr_pwm) == 7) {
            serialCommand = cmd;
            ai_servo_val = srv;
            ai_dir_val = r_dir;
            ai_motor_val = r_pwm;
            ai_dir_depan_kiri_val = fl_dir;
            ai_motor_depan_kiri_val = fl_pwm;
            ai_dir_depan_kanan_val = fr_dir;
            ai_motor_depan_kanan_val = fr_pwm;
          }
        }
        else if (cmd == 'W') {
           serialCommand = cmd;
        }
        else if (cmd == 'M') {
          if (strncmp(serialInputBuffer, "M,AUTO", 6) == 0) {
            if (isManual) { 
              isManual = false; 
              counter = 0; 
              is_new_wp = true; // --- [BARU] Reset jalur LOS saat beralih ke AUTO via Serial ---
            }
          } else if (strncmp(serialInputBuffer, "M,MANUAL", 8) == 0) {
            if (!isManual) { isManual = true; }
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

void setup() {
  Serial.begin(230400); 

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
}

void loop() {
  // --- 1. Baca Sensor GPS ---
  bool gotGPS = myGPS.getPVT();
  if (gotGPS) { 
    uint8_t fixType = myGPS.getFixType(); 
    if (fixType > 0) digitalWrite(LED_GPS, HIGH);
    else digitalWrite(LED_GPS, LOW);

    if (fixType > 0) { 
        lat = myGPS.getLatitude() / 10000000.0;
        lon = myGPS.getLongitude() / 10000000.0;
        speed = myGPS.getGroundSpeed() / 1000.0 * 3.6; 
        sats = myGPS.getSIV(); 
    } else { 
        lat = 0.0; lon = 0.0; speed = 0.0; sats = 0;
    }
  }
  
  checkSerialInput(); 
  
  float temp_heading = readCompass();
  if (temp_heading != -1) { heading = temp_heading; } 

  // --- RC INIT ---
  if (!rc_initialized) {
    if (ppm_valid[0] >= 900 && ppm_valid[0] <= 2100) { 
      valid_frame_count++;
      if (valid_frame_count > 10) rc_initialized = true; 
    }
  }

  // Baca Channel Radio (PPM)
  int ch5 = readChannel(4); 
  int ch6 = readChannel(5); 

  // Debounce for isManual
  bool rc_wants_manual = (ch5 < 1500);
  static int debounce_counter = 0;
  static bool pending_mode = rc_wants_manual;
  
  if (rc_wants_manual != isManual) {
    if (rc_wants_manual == pending_mode) {
      debounce_counter++;
      if (debounce_counter >= 3) { 
        isManual = rc_wants_manual;
        debounce_counter = 0;
        if (isManual) {
          wasInCaptureMode = false;
          wasInSaveMode = false;
        } else {
          // --- [BARU] Reset jalur saat beralih ke AUTO via Remote ---
          counter = 0;          // Kembali ke Waypoint pertama
          is_new_wp = true;     // Buat lintasan baru dari titik saat ini
        }
      }
    } else {
      pending_mode = rc_wants_manual;
      debounce_counter = 1;
    }
  } else {
    debounce_counter = 0;
  }

  String mode = "MANUAL";
  String status = "ACTIVE";
  
  int finalServo = 90;
  int finalMotor = 1500;           
  int finalMotorDepanKiri = 1000;  
  int finalMotorDepanKanan = 1000; 
  int finalDir = 1500;             
  int finalDirDepanKiri = 1500;
  int finalDirDepanKanan = 1500;
  
  int wp_target_idx = 0;
  double wp_dist_m = 0.0;
  double wp_target_brg = 0.0;
  double wp_error_hdg = 0.0;

  // ----------------- MANUAL MODE -----------------
  if (isManual) { 
    mode = "MANUAL";
    int ch1 = readChannel(0); 
    finalServo = map(ch1, 1000, 2000, 0, 180); 

    int ch3 = readChannel(2); 
    if (!throttle_armed) {
        if (ch3 <= 1050) throttle_armed = true;
        else ch3 = 1000; 
    }
    finalMotor = ch3;
    finalMotorDepanKiri = 1000;
    finalMotorDepanKanan = 1000;

    int ch8 = readChannel(7);
    finalDir = ch8;
    finalDirDepanKiri = ch8;
    finalDirDepanKanan = ch8;

    if (ch6 >= 1400 && ch6 <= 1600) { 
      if (!wasInCaptureMode) {
        wasInCaptureMode = true;
        captureTriggered = false;
      }
    } else if (ch6 > 1900) { 
      if (wasInCaptureMode && !captureTriggered) {
        if (wasInSaveMode) {
          clearAllData();
          wasInSaveMode = false;
        }
        if (dataIndex < MAX_DATA) {
          if (sats > 0) { 
            latitudes[dataIndex] = lat;
            longitudes[dataIndex] = lon;
            dataIndex++;
            saveDataToMemory();
          }
        }
        captureTriggered = true;
      }
      wasInCaptureMode = false;
    } else if (ch6 < 1100) { 
      if (!wasInSaveMode) {
        saveDataToMemory();
        wasInSaveMode = true;
      }
      wasInCaptureMode = false;
    }
  }
  // ----------------- AUTO MODE -----------------
  else { 
    mode = "AUTO";

    // --- 1. SELALU UPDATE JARAK DAN WAYPOINT ---
    if (dataIndex > 0 && lat != 0.0 && lon != 0.0) { 
      if (counter >= dataIndex) { 
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
        double targetBearing = path_angle + los_correction;
        targetBearing = fmod((targetBearing + 360.0), 360.0); 

        double errorHeading = targetBearing - heading;
        if (errorHeading > 180) errorHeading -= 360;
        if (errorHeading < -180) errorHeading += 360;
        
        wp_target_idx = counter + 1; 
        wp_dist_m = dist;
        wp_target_brg = targetBearing;
        wp_error_hdg = errorHeading;

        // Cek apakah mencapai waypoint JIKA kita dalam mode Normal Nav
        if (currentMissionState == STATE_NORMAL_NAV && dist < 1.75) {
          int reached_wp = counter + 1; 
          if (reached_wp == 12 || reached_wp == 14) {
            currentMissionState = STATE_PHOTO_LOITER;
            missionStateStartTime = millis();
          } else if (reached_wp == 17) {
            currentMissionState = STATE_DOCKING;
            missionStateStartTime = millis();
          } else {
            counter++; 
            is_new_wp = true; 
          }
        }
      }
    } else {
      if (dataIndex == 0) status = "NO_WAYPOINTS";
      else status = "GPS_INVALID";
    }

    // --- 2. TERAPKAN AKTUATOR ---
    if (currentMissionState != STATE_NORMAL_NAV) {
        unsigned long elapsed = millis() - missionStateStartTime;
        if (currentMissionState == STATE_PHOTO_LOITER) {
            status = "PHOTO_MANEUVER";
            finalServo = 90;
            finalMotor = 1000;
            finalDir = 1500;
            finalMotorDepanKiri = 1000;
            finalMotorDepanKanan = 1000;
            if (elapsed >= 3000) {
                currentMissionState = STATE_PHOTO_REVERSE;
                missionStateStartTime = millis();
            }
        } else if (currentMissionState == STATE_PHOTO_REVERSE) {
            status = "PHOTO_MANEUVER";
            finalServo = 90;
            finalMotor = 1300; 
            finalDir = 1000; // Mundur
            finalMotorDepanKiri = 1000;
            finalMotorDepanKanan = 1000;
            if (elapsed >= 2000) {
                currentMissionState = STATE_NORMAL_NAV;
                counter++; 
                is_new_wp = true;
            }
        } else if (currentMissionState == STATE_DOCKING) {
            status = "DOCKING";
            finalServo = 45;
            finalMotor = 1100;
            finalDir = 2000; // Maju
            finalMotorDepanKiri = 1200;
            finalDirDepanKiri = 2000;
            finalMotorDepanKanan = 1200;
            finalDirDepanKanan = 1000; // Asumsikan konfigurasi putar
            if (elapsed >= 20000) {
                finalMotor = 1000;
                finalMotorDepanKiri = 1000;
                finalMotorDepanKanan = 1000;
            }
        }
    } else {
        // STATE_NORMAL_NAV
        if (status == "WP_COMPLETE" || status == "NO_WAYPOINTS" || status == "GPS_INVALID") {
            finalServo = 90;
            finalMotor = 1000;
            finalMotorDepanKiri = 1000;
            finalMotorDepanKanan = 1000;
            finalDir = 1500;
            finalDirDepanKiri = 1500;
            finalDirDepanKanan = 1500;
        } else {
            if (serialCommand == 'A') {
                int calculatedServoVal = ai_servo_val;
                if (counter >= AI_SERVO_INVERSION_INDEX) {
                    calculatedServoVal = 180 - ai_servo_val; 
                    if (calculatedServoVal > 180) calculatedServoVal = 180;
                    if (calculatedServoVal < 0) calculatedServoVal = 0;
                    status = "AI_INVERTED"; 
                } else {
                    status = "AI_ACTIVE";
                }
                finalServo           = calculatedServoVal;
                finalMotor           = ai_motor_val; 
                finalMotorDepanKiri  = ai_motor_depan_kiri_val;
                finalMotorDepanKanan = ai_motor_depan_kanan_val;
                finalDir             = ai_dir_val;
                finalDirDepanKiri    = ai_dir_depan_kiri_val;
                finalDirDepanKanan   = ai_dir_depan_kanan_val;
            } else if (serialCommand == 'W') {
                status = "WAYPOINT";
                finalDir = 1500;
                finalDirDepanKiri = 1500;
                finalDirDepanKanan = 1500;
                finalServo = PID_servo(wp_target_brg, heading);
                finalMotor = readChannel(6);          
                finalMotorDepanKiri = 1000;        
                finalMotorDepanKanan = 1000;  
            }
        }
    }
  }

  // ========================================
  // --- 3. Kontrol Aktuator Lanjutan ---
  // ========================================
  static int last_srv = -1;
  static int last_mtr = -1;
  static int last_mdk = -1;
  static int last_mdk_r = -1;
  static int last_dir = -1;
  static int last_ddk = -1;
  static int last_ddk_r = -1;

  if (finalServo != last_srv) { servoKiri.write(finalServo); servoKanan.write(finalServo); last_srv = finalServo; }
  
  if (finalDirDepanKiri != last_ddk) { dirDepanKiri.writeMicroseconds(finalDirDepanKiri); last_ddk = finalDirDepanKiri; }
  if (finalDirDepanKanan != last_ddk_r) { dirDepanKanan.writeMicroseconds(finalDirDepanKanan); last_ddk_r = finalDirDepanKanan; }
  if (finalDir != last_dir) { dirBawahKiri.writeMicroseconds(finalDir); dirBawahKanan.writeMicroseconds(finalDir); last_dir = finalDir; }
  
  if (finalMotorDepanKiri != last_mdk) { escDepanKiri.writeMicroseconds(finalMotorDepanKiri); last_mdk = finalMotorDepanKiri; }
  if (finalMotorDepanKanan != last_mdk_r) { escDepanKanan.writeMicroseconds(finalMotorDepanKanan); last_mdk_r = finalMotorDepanKanan; }
  if (finalMotor != last_mtr) { escBawahKiri.writeMicroseconds(finalMotor); escBawahKanan.writeMicroseconds(finalMotor); last_mtr = finalMotor; }

  // ========================================
  // --- 4. BLOK TELEMETRI JSON ---
  // ========================================
  jsonDoc.clear(); 
  jsonDoc["mode"] = mode;
  jsonDoc["status"] = status;
  jsonDoc["heading"] = (float)round(heading * 100) / 100;
  jsonDoc["lat"] = lat;
  jsonDoc["lon"] = lon;
  jsonDoc["speed_kmh"] = (float)round(speed * 100) / 100;
  jsonDoc["sats"] = sats;
  jsonDoc["servo_out"] = finalServo;
  jsonDoc["motor_bwh_out"] = finalMotor;
  jsonDoc["motor_d_kiri_out"] = finalMotorDepanKiri;
  jsonDoc["motor_d_kanan_out"] = finalMotorDepanKanan;
  
  if (serialCommand == 'A') {
    jsonDoc["ai_inversion_active"] = (counter >= AI_SERVO_INVERSION_INDEX);
    jsonDoc["ai_wp_target"] = counter + 1;
    jsonDoc["ai_wp_start_invert"] = AI_SERVO_INVERSION_INDEX + 1;
  }

  if (mode == "AUTO" && serialCommand == 'W') { 
    jsonDoc["wp_target_idx"] = wp_target_idx;
    if (status == "WP_COMPLETE") {
      jsonDoc["wp_dist_m"] = 0.0;
      jsonDoc["wp_target_brg"] = 0.0;
      jsonDoc["wp_error_hdg"] = 0.0;
      jsonDoc["xte_m"] = 0.0; // --- [BARU] XTE menjadi 0 saat tiba ---
    } else {
      jsonDoc["wp_dist_m"] = (float)round(wp_dist_m * 100) / 100;
      jsonDoc["wp_target_brg"] = (float)round(wp_target_brg * 100) / 100;
      jsonDoc["wp_error_hdg"] = (float)round(wp_error_hdg * 100) / 100;
      jsonDoc["xte_m"] = (float)round(cross_track_error * 100) / 100; // --- [BARU] XTE dikirim ke Telemetri ---
    }
  }

  serializeJson(jsonDoc, Serial);
  Serial.println(); 
  
  delay(80); // Sekitar ~20Hz
}