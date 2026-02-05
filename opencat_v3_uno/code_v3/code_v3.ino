#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50

/************ SERVO CENTER (CALIBRATED) ************/
int servoCenter[16] = {
  1500, 1625, 0, 0,   // FL Hip, FL Knee
  1500, 1500, 0, 0,   // FR Hip, FR Knee
  1500, 1445, 0, 0,   // BR Hip, BR Knee
  1500, 1500, 0, 0    // BL Hip, BL Knee
};

/************ LEG STATE ************/
float x[4], c[4];     // current position
float tx[4], tc[4];   // target position

/************ BLUETOOTH ************/
char btCmd = 'S';
bool isWalking = false;
unsigned long lastStepTime = 0;
int stepIndex = 0;

/************ FUNCTION DECLARATIONS ************/
void IK();
void writeServo(int ch, int offset);
void smoothMove(int durationMs);
void walkForward();
void walkBackward();
void turnLeft();
void turnRight();

/*************************************************************/

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(30);

  // Initial stand pose
  for (int i = 0; i < 4; i++) {
    x[i] = tx[i] = 0;
    c[i] = tc[i] = 140;
  }

  smoothMove(800);   // smooth stand-up
}

/*************************************************************/

void loop() {

  // -------- Bluetooth Read --------
  if (Serial.available()) {
    btCmd = Serial.read();

    if (btCmd == 'F' || btCmd == 'B') isWalking = true;
    if (btCmd == 'S') isWalking = false;
  }

  // -------- Motion State Machine --------
  if (isWalking) {
    if (btCmd == 'F') walkForward();
    if (btCmd == 'B') walkBackward();
  }

  if (btCmd == 'L') turnLeft();
  if (btCmd == 'R') turnRight();
}

/*************************************************************/
// ---------------- WALKING (STABLE CRAWL) -------------------

void walkForward() {
  if (millis() - lastStepTime < 450) return;

  int order[4] = {0, 2, 1, 3};   // FL, BR, FR, BL
  int leg = order[stepIndex];

  // reset target
  for (int i = 0; i < 4; i++) {
    tx[i] = 0;
    tc[i] = 140;
  }

  // BODY WEIGHT SHIFT (CRITICAL FOR MOVEMENT)
  if (leg == 0) tx[1] = -10;
  if (leg == 1) tx[0] = -10;
  if (leg == 2) tx[3] = -10;
  if (leg == 3) tx[2] = -10;

  // step
  tc[leg] = 110;
  tx[leg] = 35;

  smoothMove(320);

  stepIndex = (stepIndex + 1) % 4;
  lastStepTime = millis();
}

void walkBackward() {
  if (millis() - lastStepTime < 450) return;

  int order[4] = {3, 1, 2, 0};   // BL, FR, BR, FL
  int leg = order[stepIndex];

  for (int i = 0; i < 4; i++) {
    tx[i] = 0;
    tc[i] = 140;
  }

  if (leg == 0) tx[1] = 10;
  if (leg == 1) tx[0] = 10;
  if (leg == 2) tx[3] = 10;
  if (leg == 3) tx[2] = 10;

  tc[leg] = 110;
  tx[leg] = -35;

  smoothMove(320);

  stepIndex = (stepIndex + 1) % 4;
  lastStepTime = millis();
}

/*************************************************************/
// ---------------- TURNING -------------------

void turnLeft() {
  tx[0] = 25;  tc[0] = 110;
  tx[3] = -25; tc[3] = 110;
  tx[1] = tx[2] = 0;
  tc[1] = tc[2] = 140;
  smoothMove(300);
}

void turnRight() {
  tx[1] = 25;  tc[1] = 110;
  tx[2] = -25; tc[2] = 110;
  tx[0] = tx[3] = 0;
  tc[0] = tc[3] = 140;
  smoothMove(300);
}

/*************************************************************/
// ---------------- SMOOTH INTERPOLATION ---------------------

void smoothMove(int durationMs) {
  int steps = durationMs / 20;
  if (steps < 1) steps = 1;

  float sx[4], sc[4];
  for (int i = 0; i < 4; i++) {
    sx[i] = x[i];
    sc[i] = c[i];
  }

  for (int s = 0; s <= steps; s++) {
    float t = (float)s / steps;
    t = t * t * (3 - 2 * t);   // smoothstep easing

    for (int i = 0; i < 4; i++) {
      x[i] = sx[i] + (tx[i] - sx[i]) * t;
      c[i] = sc[i] + (tc[i] - sc[i]) * t;
    }

    IK();
    delay(20);
  }
}

/*************************************************************/
// ---------------- PCA9685 SERVO WRITE ----------------------

void writeServo(int channel, int offset) {
  int pulse = servoCenter[channel] + offset;
  pulse = constrain(pulse, 500, 2500);
  pwm.writeMicroseconds(channel, pulse);
}

/*************************************************************/
// ---------------- 2-DOF INVERSE KINEMATICS -----------------

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

    if (n == 0) { hipServo = 0;  kneeServo = 1;  }   // FL
    if (n == 1) { hipServo = 4;  kneeServo = 5;  }   // FR
    if (n == 2) { hipServo = 8;  kneeServo = 9;  }   // BR
    if (n == 3) { hipServo = 12; kneeServo = 13; }   // BL

    writeServo(hipServo,  hip  * 10);
    writeServo(kneeServo, knee * 10);
  }
}
