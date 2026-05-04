/*
 * Velocity Lab — Activity 1: Open Loop
 * MET 382 Industrial Instrumentation and Controls
 *
 * Fixed PWM output — no feedback, no correction.
 * Apply load to shaft and observe velocity drop in Serial Plotter.
 *
 * Pins:
 *   IN1 -> GPIO 26 | IN2 -> GPIO 25 | ENA -> GPIO 27
 *   ENC A -> GPIO 32 | ENC B -> GPIO 33
 */

// ── Pins ─────────────────────────────────────────────────────────
#define PIN_IN1   26
#define PIN_IN2   25
#define PIN_ENA   27
#define PIN_ENC_A 33
#define PIN_ENC_B 32

// ── PWM Config ───────────────────────────────────────────────────
#define PWM_FREQ  20000
#define PWM_RES   8

// ── Open Loop Speed Setting ──────────────────────────────────────
// Adjust this value (0-255) to set motor speed
// Start around 150 — enough speed to show a clear drop under load
int pwmDuty = 150;

// ── Velocity Measurement ─────────────────────────────────────────
#define VELOCITY_INTERVAL_MS 100  // measure velocity every 100ms

volatile long encoderPos  = 0;
long          lastEncPos  = 0;
double        velocity    = 0.0;  // counts per second

unsigned long lastVelTime = 0;

// ════════════════════════════════════════════════════════════════
// ENCODER ISRs
// ════════════════════════════════════════════════════════════════
void IRAM_ATTR doEncoderA() {
  if (digitalRead(PIN_ENC_A) == HIGH) {
    encoderPos += (digitalRead(PIN_ENC_B) == LOW) ? 1 : -1;
  } else {
    encoderPos += (digitalRead(PIN_ENC_B) == HIGH) ? 1 : -1;
  }
}

void IRAM_ATTR doEncoderB() {
  if (digitalRead(PIN_ENC_B) == HIGH) {
    encoderPos += (digitalRead(PIN_ENC_A) == HIGH) ? 1 : -1;
  } else {
    encoderPos += (digitalRead(PIN_ENC_A) == LOW)  ? 1 : -1;
  }
}

// ════════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), doEncoderB, CHANGE);

  // Fixed direction — forward
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, pwmDuty);

  // Serial Plotter header
  Serial.println("Setpoint,Velocity");
}

// ════════════════════════════════════════════════════════════════
// LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  if (now - lastVelTime >= VELOCITY_INTERVAL_MS) {
    double dt = (now - lastVelTime) / 1000.0;
    lastVelTime = now;

    // ── Velocity Calculation ──────────────────────────────────
    // counts per second = change in counts / elapsed time
    long currentPos = encoderPos;
    long deltaCounts = currentPos - lastEncPos;
    lastEncPos = currentPos;

    velocity = deltaCounts / dt;

    // Plot a flat "setpoint" line so students can see the drop clearly
    // Value is the no-load velocity — measure this first and set it here
    double velocitySetpoint = 800.0;  // update after measuring no-load speed

    // Serial Plotter — two traces: target and actual
    Serial.print("Setpoint:"); Serial.print(velocitySetpoint);
    Serial.print(",Velocity:"); Serial.println(velocity);
  }
}