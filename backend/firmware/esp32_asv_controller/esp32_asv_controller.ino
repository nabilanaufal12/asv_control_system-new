// Import library yang diperlukan
#include <ESP32Servo.h>

// --- Konfigurasi Pin ---
const int SERVO_PIN = 13;
const int MOTOR_PIN = 12;

// Buat objek untuk Servo dan Motor
Servo servoKemudi;
Servo motorESC;

// --- Variabel untuk Pengiriman Telemetri ---
unsigned long lastTelemetryTime = 0;
const long telemetryInterval = 1000;

void setup()
{
  Serial.begin(115200);

  servoKemudi.attach(SERVO_PIN, 500, 2500);
  motorESC.attach(MOTOR_PIN, 1000, 2000);

  servoKemudi.write(90);
  motorESC.writeMicroseconds(1500); // Posisi netral
  delay(1000);

  Serial.println("ESP32 Siap Menerima Perintah...");
}

void loop()
{
  if (Serial.available() > 0)
  {
    processSerialCommand();
  }

  unsigned long currentTime = millis();
  if (currentTime - lastTelemetryTime >= telemetryInterval)
  {
    lastTelemetryTime = currentTime;
    sendTelemetry();
  }
}

void processSerialCommand()
{
  String dataMasuk = Serial.readStringUntil('\n');
  dataMasuk.trim();

  int s_pos = dataMasuk.indexOf('S');
  int d_pos = dataMasuk.indexOf('D');
  int semicolon_pos = dataMasuk.indexOf(';');

  if (s_pos != -1 && d_pos != -1 && semicolon_pos != -1)
  {
    String speedString = dataMasuk.substring(s_pos + 1, semicolon_pos);
    int speedValue = speedString.toInt();

    String degreeString = dataMasuk.substring(d_pos + 1);
    int degreeValue = degreeString.toInt();

    // Batasi nilai agar aman untuk aktuator
    speedValue = constrain(speedValue, 1000, 2000); // Rentang aman PWM
    degreeValue = constrain(degreeValue, 0, 180);   // Rentang aman Sudut

    motorESC.writeMicroseconds(speedValue);
    servoKemudi.write(degreeValue);
  }
}

void sendTelemetry()
{
  // (Fungsi ini tidak berubah)
  float lat = -6.2088 + (random(-50, 50) / 10000.0);
  float lon = 106.8456 + (random(-50, 50) / 10000.0);
  int sats = random(8, 15);
  float battery = 11.5 + (random(0, 13) / 10.0);
  int compass = random(0, 360);
  float speed = 1.5 + (random(0, 20) / 10.0);

  String telemetryData = "T:";
  telemetryData += "GPS," + String(lat, 4) + "," + String(lon, 4) + "," + String(sats) + ";";
  telemetryData += "BAT," + String(battery, 1) + ";";
  telemetryData += "COMP," + String(compass) + ";";
  telemetryData += "SPD," + String(speed, 1);
  Serial.println(telemetryData);
}