#include <Wire.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// ---------------- GPS ----------------
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Sesuaikan dengan pin GPS
#define GPS_RX 15
#define GPS_TX 17

// ---------------- CMPS12 ----------------
#define CMPS12_ADDRESS 0x60
#define ANGLE_16BIT_REGISTER 2

// ---------------- Servo dan ESC ----------------
Servo rudderServo;
Servo motorESC;

// ---------------- PID ----------------
double Kp = 2.0, Ki = 0.0, Kd = 0.5;
double error, lastError = 0, integral = 0;

// ---------------- Waypoint ----------------
#define MAX_DATA 15 // Maksimal 15 titik data
Preferences preferences;

float latitudes[MAX_DATA];
float longitudes[MAX_DATA];
int dataIndex = 0;
int counter = 0; // Counter untuk navigasi waypoint

bool captureTriggered = false;
bool wasInCaptureMode = false;
bool wasInSaveMode = false;

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
  error = input - setpoint;   // 🔄 sudah dibalik biar logika servo benar

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
#define CHANNELS 6
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
char serialCommand = 'W'; // 'W' = Waypoint, 'A' = AI Mode

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Wire.begin(21, 22);

  rudderServo.attach(18);
  motorESC.attach(19);
  rudderServo.write(90);
  motorESC.writeMicroseconds(1500);

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, RISING);

  // Muat data yang tersimpan dari memori
  loadDataFromMemory();

  Serial.println("");
  Serial.println("🌍 GPS + WAYPOINT SYSTEM");
  Serial.println("================================");
  Serial.print("Data tersimpan: ");
  Serial.print(dataIndex);
  Serial.print("/");
  Serial.print(MAX_DATA);
  Serial.println(" titik");
  Serial.println("================================");
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Cek perintah dari Python
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar == 'A' || incomingChar == 'W') {
      serialCommand = incomingChar;
    }
  }

  int ch5 = readChannel(4); // Mode Selector (Manual/Auto)
  int ch6 = readChannel(5); // Data Capture/Display/Save

  // ----------------- MANUAL MODE -----------------
  if (ch5 < 1500) {
    if (!isManual) {
      Serial.println("Switching to MANUAL...");
      rudderServo.write(90);
      motorESC.writeMicroseconds(1500);
      isManual = true;
      wasInCaptureMode = false;
      wasInSaveMode = false;
    }

    // Kontrol Manual
    int ch1 = readChannel(0);
    int servoPos = map(ch1, 1000, 2000, 0, 180);
    rudderServo.write(servoPos);

    int ch3 = readChannel(2);
    motorESC.writeMicroseconds(ch3);

    // Kontrol waypoint (rekam / simpan) di mode MANUAL
    if (ch6 >= 1400 && ch6 <= 1600) {
      if (!wasInCaptureMode) {
        Serial.println("🟡 MODE REKAM: Siap merekam waypoint baru.");
        wasInCaptureMode = true;
        captureTriggered = false;
      }
    } else if (ch6 > 1900) { // Rekam waypoint
      if (wasInCaptureMode && !captureTriggered) {
        if (wasInSaveMode) {
          clearAllData();
          wasInSaveMode = false;
        }

        if (dataIndex >= MAX_DATA) {
          Serial.println("⚠ Memori penuh. Tidak bisa menambah titik lagi.");
        } else {
          if (gps.location.isValid()) {
            latitudes[dataIndex] = gps.location.lat();
            longitudes[dataIndex] = gps.location.lng();
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
    } else if (ch6 < 1100) { // Simpan data
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
      rudderServo.write(90);
      motorESC.writeMicroseconds(1500);
      isManual = false;
      counter = 0;
    }

    // PRIORITAS PERINTAH DARI PYTHON
    if (serialCommand == 'A') {
      rudderServo.write(90);
      motorESC.writeMicroseconds(1500);
      Serial.println("DATA:AUTO,AI_MODE_ACTIVATED");
    } 
    // LANJUTKAN WAYPOINT
    else if (serialCommand == 'W') {
      if (dataIndex > 0 && gps.location.isValid()) {
        if (counter >= dataIndex) {
          // Semua waypoint selesai → berhenti
          rudderServo.write(90);
          motorESC.writeMicroseconds(1000);
          Serial.println("DATA:AUTO,STOPPED");
        } else {
          double lat = gps.location.lat();
          double lon = gps.location.lng();
          double speed = gps.speed.kmph();
          int sats = gps.satellites.value();
    
          double targetLat = latitudes[counter];
          double targetLon = longitudes[counter];
    
          double dist = haversine(lat, lon, targetLat, targetLon);
          double targetBearing = bearing(lat, lon, targetLat, targetLon);
          double heading = readCompass();
    
          double errorHeading = targetBearing - heading;
          if (errorHeading > 180) errorHeading -= 360;
          if (errorHeading < -180) errorHeading += 360;
    
          int servoPos = PID_servo(targetBearing, heading);
          rudderServo.write(servoPos);
    
          int motorSpeed = 1700;
          if (dist > 3.0) motorSpeed = 1700;
          else if (dist > 1.0) motorSpeed = 1600;
          else motorSpeed = 1500;
          motorESC.writeMicroseconds(motorSpeed);
    
          if (dist < 1.5) {
            counter++;
          }
    
          Serial.print("DATA:AUTO,");
          Serial.print(counter + 1);
          Serial.print(",");
          Serial.print(dist);
          Serial.print(",");
          Serial.print(targetBearing);
          Serial.print(",");
          Serial.print(heading);
          Serial.print(",");
          Serial.print(errorHeading);
          Serial.print(",");
          Serial.print(servoPos);
          Serial.print(",");
          Serial.print(motorSpeed);
          Serial.print(",");
          Serial.print(speed);
          Serial.print(",");
          Serial.println(sats);
        }
      } else {
        rudderServo.write(90);
        motorESC.writeMicroseconds(1500);
        if (dataIndex == 0) {
          Serial.println("DATA:AUTO,NO_WAYPOINTS");
        } else {
          Serial.println("DATA:AUTO,GPS_INVALID");
        }
      }
    }
  }

  delay(250); // Interval pengiriman data
}