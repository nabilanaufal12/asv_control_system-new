/*
  ESP32 Firmware for ASV Control System
  - Reads GPS, Compass, and a 6-channel RC receiver.
  - Sends structured telemetry data to a Jetson via Serial.
  - Receives and executes actuator commands from the Jetson.
*/

// --- Pustaka yang Diperlukan ---
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h> // Untuk kompas HMC5883L atau sejenisnya
#include <Servo.h>

// --- Konfigurasi Pin ---
// Sesuaikan pin ini dengan koneksi hardware Anda
const int RC_PPM_PIN = 27; // Pin untuk sinyal PPM dari receiver RC
const int MOTOR_ESC_PIN = 14;
const int SERVO_PIN = 12;

// --- Inisialisasi Objek ---
// GPS (misalnya, menggunakan Serial2)
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// Servo & Motor
Servo motorESC;
Servo steeringServo;

// --- Variabel Global untuk RC & Telemetri ---
// Array untuk menyimpan nilai dari 6 channel RC (biasanya 1000-2000 us)
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
    // Simpan pulse width jika dalam rentang valid (biasanya 900-2100 us)
    if (pulse_width > 900 && pulse_width < 2100) {
      rc_channels[current_channel] = pulse_width;
    }
    current_channel++;
  }
}

// --- Fungsi Setup ---
void setup() {
  // Mulai komunikasi Serial utama (ke Jetson)
  Serial.begin(115200);

  // Mulai komunikasi Serial untuk GPS
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX

  // Inisialisasi I2C untuk kompas
  Wire.begin();
  // ... (tambahkan inisialisasi spesifik untuk kompas Anda di sini)

  // Attach Servo & Motor
  motorESC.attach(MOTOR_ESC_PIN, 1000, 2000); // Min/Max pulse width
  steeringServo.attach(SERVO_PIN);
  motorESC.writeMicroseconds(1500); // Inisialisasi ESC pada posisi netral
  steeringServo.write(90);

  // Pasang interrupt untuk membaca sinyal RC PPM
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

  // Baca data Kompas (ganti dengan logika pembacaan kompas Anda)
  // heading = readCompass();
}

void sendTelemetry() {
  // Format string: T:GPS,lat,lon;COMP,head;SPD,mps;RC,ch1,ch2,ch3,ch4,ch5,ch6
  String telemetry_string = "T:";
  telemetry_string += "GPS," + String(latitude, 6) + "," + String(longitude, 6) + ";";
  telemetry_string += "COMP," + String(heading, 2) + ";";
  telemetry_string += "SPD," + String(speed_mps, 2) + ";";
  
  // Tambahkan data RC
  telemetry_string += "RC,";
  for (int i = 0; i < 6; i++) {
    telemetry_string += String(rc_channels[i]);
    if (i < 5) telemetry_string += ",";
  }

  Serial.println(telemetry_string);
}

void parseAndExecuteCommand(String cmd) {
  // Format perintah dari Jetson: "S<pwm>;D<derajat>"
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
    
    // Batasi nilai untuk keamanan
    pwm_val = constrain(pwm_val, 1000, 2000);
    servo_val = constrain(servo_val, 0, 180);

    // Tulis ke aktuator
    motorESC.writeMicroseconds(pwm_val);
    steeringServo.write(servo_val);
  }
}
