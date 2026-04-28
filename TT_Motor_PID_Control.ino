/*
 * Motor Encoder PID Control — ESP32 + L298N + TT Motor
 * MET 382 Industrial Instrumentation and Controls
 *
 * Pins:
 *   IN1  -> GPIO 26  |  IN2 -> GPIO 25  |  ENA -> GPIO 27 (PWM)
 *   ENC A -> GPIO 32 |  ENC B -> GPIO 33
 */

// ── Pin Definitions ──────────────────────────────────────────────
#define PIN_IN1   26
#define PIN_IN2   25
#define PIN_ENA   27
#define PIN_ENC_A 33
#define PIN_ENC_B 32

// ── PWM Config ───────────────────────────────────────────────────
#define PWM_FREQ   20000  // 20kHz — silent
#define PWM_RES    8      // 8-bit: 0-255

// ── PID Sample Rate ──────────────────────────────────────────────
#define PID_INTERVAL_MS 10  // 100 Hz — run PID every 10ms

// ── PID Gains — tune these ───────────────────────────────────────
double Kp = 0.3;
double Ki = 0.0;
double Kd = 0.0;

// ── PID Internal State ───────────────────────────────────────────
double setpoint    = 0.0;   // target position (encoder counts)
double input       = 0.0;   // measured position (encoder counts)
double output      = 0.0;   // PID output (-255 to 255)

double error       = 0.0;   // current error = setpoint - input
double lastError   = 0.0;   // previous error (for derivative)
double integral    = 0.0;   // running sum of error over time

// Anti-windup: clamp integral so it can't grow unbounded
#define INTEGRAL_LIMIT 5000.0

// ── Encoder ──────────────────────────────────────────────────────
volatile long encoderPos = 0;

// ── Timing ───────────────────────────────────────────────────────
unsigned long lastPIDTime = 0;
unsigned long lastPlotTime = 0;

// ── Serial Input ─────────────────────────────────────────────────
String cmdBuffer = "";
double demand = 0.0;  // commanded setpoint from serial

// ════════════════════════════════════════════════════════════════
// ENCODER ISRs — full quadrature (4x resolution)
// IRAM_ATTR keeps ISR in RAM — required for reliability on ESP32
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
// MOTOR DRIVER
// ════════════════════════════════════════════════════════════════
void setMotor(double pwmVal) {
  int duty = (int)constrain(abs(pwmVal), 0, 255);

  if (pwmVal > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(PIN_ENA, duty);
  } else if (pwmVal < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    ledcWrite(PIN_ENA, duty);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    ledcWrite(PIN_ENA, 0);
  }
}

// ════════════════════════════════════════════════════════════════
// PID ALGORITHM — the whole thing, fully visible
// Called every PID_INTERVAL_MS milliseconds
// ════════════════════════════════════════════════════════════════
void computePID() {
  double dt = PID_INTERVAL_MS / 1000.0;  // time step in seconds (0.01 s)

  // ── 1. ERROR ─────────────────────────────────────────────────
  // How far are we from where we want to be?
  error = setpoint - input;

  // ── 2. PROPORTIONAL ──────────────────────────────────────────
  // Output proportional to current error.
  // Large error -> large correction. Zero error -> zero P output.
  double P = Kp * error;

  // ── 3. INTEGRAL ───────────────────────────────────────────────
  // Accumulate error over time to eliminate steady-state error.
  // If the motor stalls just short of target, integral builds up
  // and pushes harder until error is zero.
  integral += error * dt;

  // Anti-windup: prevent integral from growing forever if motor
  // is saturated or blocked (would cause overshoot when released)
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  double I = Ki * integral;

  // ── 4. DERIVATIVE ─────────────────────────────────────────────
  // React to the RATE of change of error.
  // If error is shrinking fast, D term backs off the output —
  // this damps overshoot. Acts like a brake as we approach target.
  double derivative = (error - lastError) / dt;
  double D = Kd * derivative;

  // ── 5. TOTAL OUTPUT ───────────────────────────────────────────
  // Sum of all three terms. Sign determines motor direction.
  output = P + I + D;

  // Clamp output to motor driver range
  output = constrain(output, -255, 255);

  // Save error for next derivative calculation
  lastError = error;
}

// ════════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  // Motor output pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  // PWM on ENA
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES);

  // Encoder input pins with pullups
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), doEncoderB, CHANGE);

  setMotor(0);

  Serial.println("=== MET 382 PID Motor Control ===");
  Serial.println("Commands (send via Serial Monitor):");
  Serial.println("  a        -> setpoint = 500 counts");
  Serial.println("  z        -> setpoint = 0 (return home)");
  Serial.println("  p<value> -> set Kp  e.g. p0.5");
  Serial.println("  i<value> -> set Ki  e.g. i0.01");
  Serial.println("  d<value> -> set Kd  e.g. d0.05");
  Serial.println("  s<value> -> set custom setpoint  e.g. s750");
  Serial.println("  r        -> reset encoder to zero");
  Serial.println();
  Serial.println("Encoder, Command");  // Serial Plotter header
}

// ════════════════════════════════════════════════════════════════
// LOOP
// ════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── Serial command handling ───────────────────────────────────
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuffer.length() > 0) {
        char  cmd = cmdBuffer.charAt(0);
        float val = cmdBuffer.substring(1).toFloat();

        switch (cmd) {
          case 'a':
            demand = 500;
            integral = 0;  // reset integral on new setpoint
            Serial.println("Setpoint -> 500");
            break;
          case 'z':
            demand = 0;
            integral = 0;
            Serial.println("Setpoint -> 0");
            break;
          case 's':
            demand = val;
            integral = 0;
            Serial.print("Setpoint -> "); Serial.println(demand);
            break;
          case 'p':
            Kp = val;
            Serial.print("Kp -> "); Serial.println(Kp);
            break;
          case 'i':
            Ki = val;
            integral = 0;
            Serial.print("Ki -> "); Serial.println(Ki);
            break;
          case 'd':
            Kd = val;
            Serial.print("Kd -> "); Serial.println(Kd);
            break;
          case 'r':
            encoderPos = 0;
            integral   = 0;
            lastError  = 0;
            demand     = 0;
            setMotor(0);
            Serial.println("Encoder reset.");
            break;
        }
        // Print current gains after any command
        Serial.printf("Gains -> Kp: %.4f  Ki: %.4f  Kd: %.4f\n", Kp, Ki, Kd);
        cmdBuffer = "";
      }
    } else {
      cmdBuffer += c;
    }
  }

  // ── PID loop — runs every PID_INTERVAL_MS ────────────────────
  if (now - lastPIDTime >= PID_INTERVAL_MS) {
    lastPIDTime = now;

    setpoint = demand;
    input    = (double)encoderPos;  // snapshot volatile

    computePID();
    setMotor(output);

    // Serial Plotter output — 10x slower than PID to avoid flooding
    if (now - lastPlotTime >= 100) {
      lastPlotTime = now;
      Serial.print("Encoder:"); Serial.print(input);
      Serial.print(", Command:"); Serial.println(output);
    }
  }
}
