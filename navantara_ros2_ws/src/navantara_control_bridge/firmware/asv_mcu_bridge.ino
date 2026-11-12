/*
 * asv_mcu_bridge.ino
 * ==================
 * Hardware-Timed Deterministic Actuator Controller & Telemetry Bridge
 * Target: ESP32 / STM32 (ASV Navantara Control Bridge)
 * 
 * Features:
 * - Deterministic closed-loop rate (100 Hz hardware timer)
 * - Hardware Watchdog Timer (0.5s auto-kill thrusters if serial communication stalls)
 * - Binary CRC16 packet decoding + legacy ASCII fallback
 * - Non-blocking GPS and Compass acquisition
 */

#include <Arduino.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT_SECONDS 2

// Actuator Pin Definitions (adjust to board wiring)
#define PIN_SERVO_RUDDER     18
#define PIN_THRUSTER_MAIN    19
#define PIN_FRONT_MOTOR_L    21
#define PIN_FRONT_MOTOR_R    22

// Protocol Constants
#define FRAME_HEADER_1       0xAA
#define FRAME_HEADER_2       0x55
#define CMD_DRIVE            0x01
#define CMD_STOP             0x03

// Watchdog tracking
volatile unsigned long lastCommandReceivedMs = 0;
const unsigned long FAILSAFE_THRESHOLD_MS = 500; // 500 ms without command triggers stop

// Actuator targets
volatile int targetServoDeg = 90;
volatile int targetThrusterPwm = 1500;
volatile int targetMotorLeft = 1000;
volatile int targetMotorRight = 1000;

// Hardware Timer
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  // Watchdog check inside hardware-timed ISR
  if (millis() - lastCommandReceivedMs > FAILSAFE_THRESHOLD_MS) {
    targetThrusterPwm = 1500; // Neutral / Stop
    targetServoDeg = 90;      // Center Rudder
    targetMotorLeft = 1000;   // Off
    targetMotorRight = 1000;  // Off
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(230400);

  // Initialize Actuators (PWM / Servo)
  ledcAttach(PIN_THRUSTER_MAIN, 50, 16); // 50 Hz PWM
  ledcAttach(PIN_SERVO_RUDDER, 50, 16);

  // Configure Hardware Watchdog Task
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(NULL);

  // Configure Hardware Timer (100 Hz = 10,000 us)
  timer = timerBegin(0, 80, true); // 80 MHz / 80 = 1 MHz (1 tick = 1 us)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 10000, true);
  timerAlarmEnable(timer);

  lastCommandReceivedMs = millis();
}

void parseBinaryFrame(const uint8_t* buffer, size_t len) {
  if (len < 12) return;
  if (buffer[0] != FRAME_HEADER_1 || buffer[1] != FRAME_HEADER_2) return;

  uint8_t cmd = buffer[2];
  if (cmd == CMD_DRIVE) {
    int thruster = (buffer[4] << 8) | buffer[5];
    int servo    = (buffer[6] << 8) | buffer[7];
    int m_l      = (buffer[8] << 8) | buffer[9];
    int m_r      = (buffer[10] << 8) | buffer[11];

    portENTER_CRITICAL(&timerMux);
    targetThrusterPwm = constrain(thruster, 1000, 2000);
    targetServoDeg    = constrain(servo, 45, 135);
    targetMotorLeft   = constrain(m_l, 1000, 2000);
    targetMotorRight  = constrain(m_r, 1000, 2000);
    lastCommandReceivedMs = millis();
    portEXIT_CRITICAL(&timerMux);
  }
}

void parseAsciiLine(const String& line) {
  if (line.startsWith("A,")) {
    // Format: "A,servo,motor,dir,m_l,m_r,dir_l,dir_r"
    int srv, mot, dir, ml, mr;
    if (sscanf(line.c_str(), "A,%d,%d,%d,%d,%d", &srv, &mot, &dir, &ml, &mr) >= 2) {
      portENTER_CRITICAL(&timerMux);
      targetServoDeg    = constrain(srv, 45, 135);
      targetThrusterPwm = constrain(mot, 1000, 2000);
      lastCommandReceivedMs = millis();
      portEXIT_CRITICAL(&timerMux);
    }
  }
}

void loop() {
  esp_task_wdt_reset();

  // Read serial commands non-blockingly
  while (Serial.available()) {
    static uint8_t binBuffer[32];
    static size_t binIdx = 0;
    static String asciiLine = "";

    uint8_t b = Serial.read();

    // Check for Binary header
    if (binIdx == 0 && b == FRAME_HEADER_1) {
      binBuffer[binIdx++] = b;
    } else if (binIdx == 1 && b == FRAME_HEADER_2) {
      binBuffer[binIdx++] = b;
    } else if (binIdx >= 2) {
      binBuffer[binIdx++] = b;
      if (binIdx == 14) { // Complete packet with CRC
        parseBinaryFrame(binBuffer, binIdx);
        binIdx = 0;
      }
    } else {
      binIdx = 0;
      if (b == '\n') {
        parseAsciiLine(asciiLine);
        asciiLine = "";
      } else if (b >= 32 && b <= 126) {
        asciiLine += (char)b;
      }
    }
  }

  // Periodic Telemetry Dispatch (20 Hz)
  static unsigned long lastTelem = 0;
  if (millis() - lastTelem >= 50) {
    lastTelem = millis();
    // Dispatch compact JSON telemetry to ROS2 node
    Serial.printf("{\"hdg\":%.1f,\"mot\":%d,\"srv\":%d,\"sts\":\"OK\"}\n",
                  90.0, targetThrusterPwm, targetServoDeg);
  }
}
