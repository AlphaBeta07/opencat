#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50

int angle[16];
int servoref[16] = {
  90, 90, 0, 0,
  90, 90, 0, 0,
  90, 90, 0, 0,
  90, 90, 0, 0
};

float c[4], d[4], x[4], z[4];

// Bluetooth state
char btCmd = 'S';
unsigned long lastStepTime = 0;
int stepIndex = 0;

void holdposition();
void IK();

// ----------------------------------------------------

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Default stand
  for (int i = 0; i < 4; i++) {
    c[i] = 140;
    x[i] = 0;
  }

  Serial.println("Bluetooth Quadruped Ready");
}

// ----------------------------------------------------

void loop() {

  // Read Bluetooth
  if (Serial.available()) {
    btCmd = Serial.read();
  }

  // Command handler
  switch (btCmd) {
    case 'A': standPose(); break;
    case 'M': sitPose();   break;
    case 'D': layPose();   break;
    case 'C': sleepPose(); break;

    case 'F': walkForward();  break;
    case 'B': walkBackward(); break;
    case 'L': turnLeft();     break;
    case 'R': turnRight();    break;

    case 'S': stopMove();     break;
  }
}

// ----------------------------------------------------
// POSTURES

void standPose() {
  for (int i = 0; i < 4; i++) {
    x[i] = 0;
    c[i] = 140;
  }
  IK(); holdposition();
}

void sitPose() {
  c[0] = c[1] = 160;
  c[2] = c[3] = 110;
  IK(); holdposition();
}

void layPose() {
  for (int i = 0; i < 4; i++) {
    c[i] = 90;
  }
  IK(); holdposition();
}

void sleepPose() {
  layPose();
  btCmd = 'S';
}

// ----------------------------------------------------
// WALKING (CRAWL)

void walkForward() {
  if (millis() - lastStepTime < 300) return;

  int order[4] = {0, 2, 1, 3}; // FL, BR, FR, BL
  int leg = order[stepIndex];

  for (int i = 0; i < 4; i++) {
    x[i] = 0;
    c[i] = 140;
  }

  c[leg] = 110;
  x[leg] = 30;

  IK(); holdposition();

  stepIndex = (stepIndex + 1) % 4;
  lastStepTime = millis();
}

void walkBackward() {
  if (millis() - lastStepTime < 300) return;

  int order[4] = {3, 1, 2, 0};
  int leg = order[stepIndex];

  for (int i = 0; i < 4; i++) {
    x[i] = 0;
    c[i] = 140;
  }

  c[leg] = 110;
  x[leg] = -30;

  IK(); holdposition();

  stepIndex = (stepIndex + 1) % 4;
  lastStepTime = millis();
}

void turnLeft() {
  if (millis() - lastStepTime < 300) return;

  c[0] = 110; x[0] = 20;
  c[3] = 110; x[3] = -20;

  IK(); holdposition();
  lastStepTime = millis();
}

void turnRight() {
  if (millis() - lastStepTime < 300) return;

  c[1] = 110; x[1] = 20;
  c[2] = 110; x[2] = -20;

  IK(); holdposition();
  lastStepTime = millis();
}

void stopMove() {
  // hold position
}

// ----------------------------------------------------
// SERVO OUTPUT

void holdposition() {
  for (int i = 0; i < 16; i++) {
    int us = map(angle[i], 0, 180, 500, 2500);
    pwm.writeMicroseconds(i, us);
  }
}

// ----------------------------------------------------
// 2-DOF INVERSE KINEMATICS (UNCHANGED, CLEAN)

void IK() {
  float a = 80;
  float b = 140;

  for (int n = 0; n < 4; n++) {
    float D = sqrt(x[n]*x[n] + c[n]*c[n]);
    D = constrain(D, abs(a - b), a + b);

    float knee = acos((a*a + b*b - D*D) / (2*a*b));
    float hip  = atan2(c[n], x[n]) -
                 acos((a*a + D*D - b*b) / (2*a*D));

    knee *= 180 / PI;
    hip  *= 180 / PI;

    int hipServo, kneeServo;
    if (n == 0) { hipServo = 0;  kneeServo = 1;  }
    if (n == 1) { hipServo = 4;  kneeServo = 5;  }
    if (n == 2) { hipServo = 8;  kneeServo = 9;  }
    if (n == 3) { hipServo = 12; kneeServo = 13; }

    angle[hipServo]  = servoref[hipServo]  + hip;
    angle[kneeServo] = servoref[kneeServo] + knee;
  }
}
