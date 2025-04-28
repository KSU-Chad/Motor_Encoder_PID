#include <PID_v1.h>

double Pk1 = .2;
double Ik1 = 0;
double Dk1 = 0.0;

double Setpoint1, Input1, Output1, Output1a;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT);

float demand1;

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long standbyMillis;
unsigned long graphMillis;

#define encoder0PinA 32
#define encoder0PinB 33

volatile long encoder0Pos = 0;

int motorPin1 = 25;
int motorPin2 = 26;
int pwmPin1 = 27;

const int freq = 500;
const int ledChannel1 = 1;
const int resolution = 8;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(pwmPin1, OUTPUT);

  ledcSetup(ledChannel1, freq, resolution);

  ledcAttachPin(pwmPin1, ledChannel1);

  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE);

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(10);

  Serial.begin(115200);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'a') {
        demand1 = 500;
        //Serial.println("Received 'a' ***********");
        standbyMillis = currentMillis;
      } else if (c == 'z') {
        demand1 = 0;
        //Serial.println("Received 'z' ***********");
        standbyMillis = currentMillis;
      }
    }
    //if (currentMillis - standbyMillis <= 4900) {
      Setpoint1 = demand1;
      Input1 = encoder0Pos;
      PID1.Compute();
      if (currentMillis - graphMillis >= 100) {
        graphMillis = currentMillis;
        Serial.print("Encoder:");
        Serial.print(encoder0Pos);
        Serial.print(", Command:");
        Serial.println(Output1);
      }
    //} else {
     // Output1 = 0;
    //}
    //motor1
    if (Output1 > 0) {  //90??
      Output1a = abs(Output1);
      ledcWrite(ledChannel1, Output1a);
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    } else if (Output1 < 0) {  //-90??
      Output1a = abs(Output1);
      ledcWrite(ledChannel1, Output1a);
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
    } else {
      ledcWrite(ledChannel1, 0);
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
    }
  }
}
void doEncoderA() {
  if (digitalRead(encoder0PinA) == HIGH) {
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;
    } else {
      encoder0Pos = encoder0Pos - 1;
    }
  } else {
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;
    } else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
}
void doEncoderB() {
  if (digitalRead(encoder0PinB) == HIGH) {
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;
    } else {
      encoder0Pos = encoder0Pos - 1;
    }
  } else {
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;
    } else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
}
