/*
  ESP32 Firmware for ASV Control System
  - Reads GPS, Compass, and a 6-channel RC receiver.
  - Sends structured telemetry data to a Jetson via Serial.
  - Receives and executes actuator commands from the Jetson.
  - Version: 1.1 (Refined with ESP32Servo best practices)
*/

// --- Pustaka yang Diperlukan ---
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h> // Untuk kompas HMC5883L atau sejenisnya
#include <ESP32Servo.h> // Pustaka yang dioptimalkan untuk ESP32

// --- Konfigurasi Pin ---
const int RC_PPM_PIN = 27;    // Pin untuk sinyal PPM dari receiver RC
const int MOTOR_ESC_PIN = 14;   // Pin ESC motor
const int SERVO_PIN = 12;       // Pin servo kemudi

// --- Inisialisasi Objek ---
Servo motorESC;
Servo steeringServo;
HardwareSerial gpsSerial(2);  // Gunakan UART2 untuk GPS
TinyGPSPlus gps;

// --- Variabel Global untuk RC & Telemetri ---
volatile int rc_channels[6] = {1500, 1500, 1500, 1500, 1500, 1500};
volatile int current_channel = 0;
volatile unsigned long last_rc_pulse_time = 0;

// Variabel untuk data sensor
float latitude = -6.9180;
float longitude = 107.6185;
float heading = 90.0;
float speed_mps = 0.0;

// --- Pengaturan Waktu ---
unsigned long last_telemetry_send_time = 0;
const long TELEMETRY_INTERVAL = 200; // Kirim data setiap 200 ms (5 Hz)

// --- Fungsi Interrupt untuk Membaca RC PPM ---
void IRAM_ATTR read_rc_ppm() {
  unsigned long now = micros();
  unsigned long pulse_width = now - last_rc_pulse_time;
  last_rc_pulse_time = now;

  if (pulse_width > 4000) { // Sync pulse, reset channel counter
    current_channel = 0;
  } else if (current_channel < 6) {
    if (pulse_width > 900 && pulse_width < 2100) {
      rc_channels[current_channel] = pulse_width;
    }
    current_channel++;
  }
}

// --- Fungsi Setup ---
void setup() {
  Serial.begin(115200); // Serial utama ke Jetson
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX

  // Inisialisasi I2C untuk kompas (Wire)
  Wire.begin();

  // Attach Servo & Motor dengan batas pulsa
  motorESC.setPeriodHertz(50); // Frekuensi 50 Hz umum untuk ESC dan servo
  steeringServo.setPeriodHertz(50);
  motorESC.attach(MOTOR_ESC_PIN, 1000, 2000); // Tentukan rentang pulsa (min/max)
  steeringServo.attach(SERVO_PIN, 1000, 2000); // Tentukan rentang pulsa (min/max)

  // Inisialisasi posisi netral
  motorESC.writeMicroseconds(1500);
  // REFINEMENT: Gunakan writeMicroseconds untuk konsistensi. 1500 us adalah standar tengah untuk servo.
  steeringServo.writeMicroseconds(1500);

  // Inisialisasi pembacaan PPM
  pinMode(RC_PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RC_PPM_PIN), read_rc_ppm, FALLING);

  Serial.println("ESP32 ASV Controller Initialized.");
}

// --- Loop Utama ---
void loop() {
  // 1. Baca dan proses data dari Jetson
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseAndExecuteCommand(command);
  }

  // 2. Baca dan proses data dari sensor
  updateSensors();

  // 3. Kirim data telemetri ke Jetson secara berkala
  if (millis() - last_telemetry_send_time > TELEMETRY_INTERVAL) {
    sendTelemetry();
    last_telemetry_send_time = millis();
  }
}

// --- Fungsi Pembantu ---
void updateSensors() {
  // Baca data GPS
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
      if (gps.speed.isValid()) {
        speed_mps = gps.speed.mps();
      }
    }
  }

  // Baca data Kompas (implementasi disesuaikan dengan sensor Anda)
  // heading = readCompass(); // placeholder
}

void sendTelemetry() {
  // Format: T:GPS,lat,lon;COMP,head;SPD,mps;RC,ch1,ch2,ch3,ch4,ch5,ch6
  String telemetry_string = "T:";
  telemetry_string += "GPS," + String(latitude, 6) + "," + String(longitude, 6) + ";";
  telemetry_string += "COMP," + String(heading, 2) + ";";
  telemetry_string += "SPD," + String(speed_mps, 2) + ";";

  telemetry_string += "RC,";
  for (int i = 0; i < 6; i++) {
    telemetry_string += String(rc_channels[i]);
    if (i < 5) telemetry_string += ",";
  }

  Serial.println(telemetry_string);
}

void parseAndExecuteCommand(String cmd) {
  // Format perintah: "S<pwm>;D<derajat>"
  // Contoh: "S1600;D75"
  cmd.trim();
  int s_pos = cmd.indexOf('S');
  int d_pos = cmd.indexOf('D');
  int semi_pos = cmd.indexOf(';');

  if (s_pos != -1 && d_pos != -1) {
    String pwm_str = cmd.substring(s_pos + 1, semi_pos);
    String servo_str = cmd.substring(d_pos + 1);
    
    int pwm_val = pwm_str.toInt();
    int servo_val = servo_str.toInt();

    pwm_val = constrain(pwm_val, 1000, 2000);
    
    // REFINEMENT: Ubah logika untuk menerima derajat (0-180) dan mengonversinya ke microseconds
    // Ini membuat perintah dari Jetson lebih intuitif ("kirim derajat")
    // sementara ESP32 menangani konversi ke pulsa yang sebenarnya.
    int servo_us = map(servo_val, 0, 180, 1000, 2000);
    servo_us = constrain(servo_us, 1000, 2000);

    motorESC.writeMicroseconds(pwm_val);
    steeringServo.writeMicroseconds(servo_us);
  }
}
