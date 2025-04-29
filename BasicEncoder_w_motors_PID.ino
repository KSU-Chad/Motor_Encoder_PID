#include <PID_v1.h>

double Kp = 0.3;
double Ki = 0.0;
double Kd = 0.0;

String data_in = "";
byte received_byte;

double Setpoint1, Input1, Output1, Output1a;
PID myPID(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);

float demand1;

unsigned long currentMillis;
unsigned long previousMillis;
unsigned long graphMillis;

#define encoder0PinA 32
#define encoder0PinB 33

volatile long encoder0Pos = 0;

int motorPin1 = 25;
int motorPin2 = 26;
int pwmPin1 = 27;

const int freq = 500;
//const int ledChannel1 = 1;
const int resolution = 8;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(pwmPin1, OUTPUT);

  //ledcSetup(ledChannel1, freq, resolution);
  // ledcAttachPin(pwmPin1, ledChannel1);

  ledcAttach(pwmPin1, freq, resolution);

  pinMode(encoder0PinA, INPUT_PULLUP);
  pinMode(encoder0PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(10);

  Serial.begin(115200);
}

void loop() {
  currentMillis = millis();

  if (Serial.available() > 0) {
    received_byte = Serial.read();  //Load the received serial data in the received_byte variable
    data_in = data_in + char(received_byte);
  }
  String payload = "";
  if ((data_in != "") && (Serial.available() == 0)) {
    char value = data_in.charAt(0);
    payload = data_in.substring(1, data_in.indexOf('\n'));
    switch (value) {
      case 'p':
        Kp = payload.toFloat();  //Gain setting for the P-controller (15)
        myPID.SetTunings(Kp, Ki,Kd); 
        Serial.print("Kp: ");
        Serial.println(Kp);
        break;
      case 'i':
        Ki = payload.toFloat();  //Gain setting for the I-controller (1.5)
        myPID.SetTunings(Kp, Ki,Kd); 
        Serial.print("Ki: ");
        Serial.println(Ki);
        break;
      case 'd':
        Kd = payload.toFloat();  //Gain setting for the D-controller (30)
        myPID.SetTunings(Kp, Ki,Kd); 
        Serial.print("Kd: ");
        Serial.println(Kd);
        break;
      case 'a':
        demand1 = 500;
        break;
      case 'z':
        demand1 = 0;
        break;
    }
    data_in = "";
    Serial.println();
    Serial.print("Kp: ");
    Serial.println(Kp);
    Serial.print("Ki: ");
    Serial.println(Ki);
    Serial.print("Kd: ");
    Serial.println(Kd);
    Serial.println();
  }



  if (currentMillis - previousMillis >= 10) {
    previousMillis = currentMillis;
    Setpoint1 = demand1;
    Input1 = encoder0Pos;
    myPID.Compute();
    if (currentMillis - graphMillis >= 100) {
      graphMillis = currentMillis;
      Serial.print("Encoder:");
      Serial.print(encoder0Pos);
      Serial.print(", Command:");
      Serial.println(Output1);
    }
    if (Output1 > 0) {
      Output1a = abs(Output1);
      ledcWrite(pwmPin1, Output1a);
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    } else if (Output1 < 0) {
      Output1a = abs(Output1);
      ledcWrite(pwmPin1, Output1a);
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
    } else {
      ledcWrite(pwmPin1, 0);
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
