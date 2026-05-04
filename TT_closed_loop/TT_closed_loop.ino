/*
 * Velocity Lab — Activity 2: Closed Loop Velocity PID
 * MET 382 Industrial Instrumentation and Controls
 *
 * PID controls motor PWM to maintain constant velocity
 * regardless of load. Apply load and watch the plotter hold flat.
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

// ── PID & Velocity Sample Rate ───────────────────────────────────
#define PID_INTERVAL_MS 100  // 10 Hz — velocity needs longer window than position PID

// ── Velocity Setpoint ────────────────────────────────────────────
// Set this to your target speed in encoder counts per second.
// Tip: run Activity 1 first, note the no-load speed at PWM=150,
// then set this to ~80% of that value so PID has headroom to push.
double velocitySetpoint = 700.0;  // counts/second — adjust to your motor

// ── PID Gains ────────────────────────────────────────────────────
// Velocity PID tunes differently than position PID:
//   - Kp will be smaller (velocity error is already a rate)
//   - Ki does most of the steady-state work here
//   - Kd is often not needed — velocity is already a derivative
double Kp = 0.1;
double Ki = 0.05;
double Kd = 0.0;   // start at zero — often unnecessary for velocity

// ── PID State ────────────────────────────────────────────────────
double velocity    = 0.0;
double error       = 0.0;
double lastError   = 0.0;
double integral    = 0.0;
double output      = 0.0;

#define INTEGRAL_LIMIT  200.0   // tighter limit than position — velocity responds faster
#define OUTPUT_MIN       30.0   // minimum PWM to keep motor spinning (deadband floor)
#define OUTPUT_MAX      255.0

// ── Encoder ──────────────────────────────────────────────────────
volatile long encoderPos = 0;
long          lastEncPos = 0;

unsigned long lastPIDTime = 0;

// ── Serial Input ─────────────────────────────────────────────────
String cmdBuffer = "";

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
// PID ALGORITHM — velocity control
// Input:  measured velocity (counts/second)
// Output: PWM duty cycle (30–255, forward only)
// ════════════════════════════════════════════════════════════════
void computeVelocityPID(double dt) {

  // ── 1. ERROR ─────────────────────────────────────────────────
  // How far is actual speed from target speed?
  error = velocitySetpoint - velocity;

  // ── 2. PROPORTIONAL ──────────────────────────────────────────
  // Immediate correction proportional to speed error.
  double P = Kp * error;

  // ── 3. INTEGRAL ───────────────────────────────────────────────
  // Accumulates error over time — does the heavy lifting in
  // velocity control. Keeps pushing PWM up until speed matches.
  integral += error * dt;
  integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  double I  = Ki * integral;

  // ── 4. DERIVATIVE ─────────────────────────────────────────────
  // Rate of change of speed error. Usually left at zero for
  // velocity control — velocity measurement is already noisy
  // and D amplifies that noise.
  double derivative = (error - lastError) / dt;
  double D = Kd * derivative;

  // ── 5. TOTAL OUTPUT ───────────────────────────────────────────
  output = P + I + D;

  // Clamp to valid PWM range — floor at OUTPUT_MIN so motor
  // doesn't stall when load is light and error is small
  output = constrain(output, OUTPUT_MIN, OUTPUT_MAX);

  lastError = error;
}

// ════════════════════════════════════════════════════════════════
// MOTOR DRIVER — velocity mode is forward only
// ════════════════════════════════════════════════════════════════
void setMotor(double pwmVal) {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, (int)pwmVal);
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

  // Seed integral so motor starts moving immediately
  // rather than waiting for integral to wind up from zero
  integral = velocitySetpoint / Ki > 0 ? velocitySetpoint * 0.5 : 0;

  Serial.println("=== Velocity PID ===");
  Serial.println("Commands:");
  Serial.println("  v<value>  set velocity setpoint (counts/sec)  e.g. v700");
  Serial.println("  p<value>  set Kp");
  Serial.println("  i<value>  set Ki");
  Serial.println("  d<value>  set Kd");
  Serial.println("  x         stop motor");
  Serial.println();
  Serial.println("Setpoint,Velocity");  // Serial Plotter header
}

// ════════════════════════════════════════════════════════════════
// LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Serial commands ───────────────────────────────────────────
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() > 0) {
        char   cmd = cmdBuffer.charAt(0);
        double val = cmdBuffer.substring(1).toFloat();
        switch (cmd) {
          case 'v':
            velocitySetpoint = val;
            integral = 0;
            Serial.print("Setpoint -> "); Serial.println(velocitySetpoint);
            break;
          case 'p': Kp = val; Serial.print("Kp -> "); Serial.println(Kp); break;
          case 'i': Ki = val; integral = 0; Serial.print("Ki -> "); Serial.println(Ki); break;
          case 'd': Kd = val; Serial.print("Kd -> "); Serial.println(Kd); break;
          case 'x':
            setMotor(0);
            digitalWrite(PIN_IN1, LOW);
            Serial.println("Stopped.");
            break;
        }
        Serial.printf("Gains -> Kp:%.3f  Ki:%.3f  Kd:%.3f\n", Kp, Ki, Kd);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }

  // ── PID loop ─────────────────────────────────────────────────
  if (now - lastPIDTime >= PID_INTERVAL_MS) {
    double dt = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    // Measure velocity — counts per second over this interval
    long currentPos  = encoderPos;
    long deltaCounts = currentPos - lastEncPos;
    lastEncPos       = currentPos;
    velocity         = deltaCounts / dt;

    computeVelocityPID(dt);
    setMotor(output);

    // Serial Plotter — two traces
    Serial.print("Setpoint:"); Serial.print(velocitySetpoint);
    Serial.print(",Velocity:"); Serial.println(velocity);
  }
}