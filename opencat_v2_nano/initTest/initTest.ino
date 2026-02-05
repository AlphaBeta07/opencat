#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Servo objects
Servo s[8];

// Servo pins order: FL_HIP, FL_KNEE, FR_HIP, FR_KNEE, BL_HIP, BL_KNEE, BR_HIP, BR_KNEE
const int pin[8] = {2,3,4,5,6,7,8,9};

// Sign maps:
int hipSign[4]  = {+1, -1, +1, -1};
int kneeSign[4] = {+1, +1, -1, -1};

// Servo trim offsets:
int trim[8] = {0,0,0,0,0,0,0,0};

// State machine
enum State {STOP, STAND, SIT, LAY, SLEEP, WALK_F, WALK_B, WALK_L, WALK_R};
State currentState = STOP;

// IMU readings
float pitch = 0, roll = 0;

// Balance gain
float kPitch = 0.8;
float kRoll  = 0.7;

// Pose variables
int standHip = 90;
int standKnee = 120; // medium stand (B)

// Gait step tracking
int gaitPhase = 0;
unsigned long gaitTimer = 0;
int gaitDelay = 200;

// Function prototypes
void poseStand();
void poseSit();
void poseLay();
void poseSleep();
void stopRobot();
void gaitForward();
void gaitBackward();
void gaitLeft();
void gaitRight();
void applyServos();
void balance();
void readIMU();
void parseBT(char c);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  
  for(int i=0;i<8;i++){
    s[i].attach(pin[i]);
    s[i].write(90);
  }
  
  poseStand();
  currentState = STAND;
}

void loop() {
  if(Serial.available()) parseBT(Serial.read());
  
  readIMU();
  
  switch(currentState) {
    case STAND:  balance(); applyServos(); break;
    case SIT:    poseSit(); break;
    case LAY:    poseLay(); break;
    case SLEEP:  poseSleep(); break;
    case STOP:   stopRobot(); break;
    case WALK_F: gaitForward(); break;
    case WALK_B: gaitBackward(); break;
    case WALK_L: gaitLeft(); break;
    case WALK_R: gaitRight(); break;
  }
}

void poseStand() {
  int hip[4]  = {standHip, standHip, standHip, standHip};
  int knee[4] = {standKnee, standKnee, standKnee, standKnee};

  for(int i=0;i<4;i++) {
    s[i*2+0].write(hip[i] * hipSign[i] + 90 + trim[i*2+0]);
    s[i*2+1].write(knee[i] * kneeSign[i] + 90 + trim[i*2+1]);
  }
}

void poseSit() {
  int hip[4]  = {90, 90, 90, 90};
  int knee[4] = {100, 100, 150, 150}; // S2 style
  for(int i=0;i<4;i++) {
    s[i*2+0].write(hip[i] + trim[i*2+0]);
    s[i*2+1].write(knee[i] + trim[i*2+1]);
  }
}

void poseLay() {
  for(int i=0;i<8;i++) {
    s[i].write(90 + trim[i]);
  }
}

void poseSleep() {
  for(int i=0;i<8;i++) {
    s[i].detach(); // torque off (sleep)
  }
}

void stopRobot() {
  poseStand();
}

void parseBT(char c) {
  switch(c) {
    case 'A': currentState = STAND; break;
    case 'M': currentState = SIT; break;
    case 'D': currentState = LAY; break;
    case 'C': currentState = SLEEP; break;
    case 'S': currentState = STOP; break;
    case 'F': currentState = WALK_F; break;
    case 'B': currentState = WALK_B; break;
    case 'L': currentState = WALK_L; break;
    case 'R': currentState = WALK_R; break;
  }
}

void readIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  float p = atan2(ax, sqrt(ay*ay + az*az)) * 57.3;
  float r = atan2(ay, sqrt(ax*ax + az*az)) * 57.3;

  pitch = -p; // arrow backward
  roll  = r;
}

void balance() {
  int pitchAdj = pitch * kPitch;
  int rollAdj  = roll * kRoll;

  for(int i=0;i<4;i++) {
    int hipAng  = standHip + rollAdj * hipSign[i];
    int kneeAng = standKnee + pitchAdj * kneeSign[i];

    s[i*2+0].write(hipAng + trim[i*2+0]);
    s[i*2+1].write(kneeAng + trim[i*2+1]);
  }
}

void applyServos() {
  balance();
}
void gaitForward() {
  unsigned long now = millis();
  if(now - gaitTimer < gaitDelay) return;
  gaitTimer = now;

  int hipMove = 20;
  int kneeLift = 20;

  switch(gaitPhase) {
    case 0: // FL
      s[0].write(standHip + hipMove * hipSign[0] + trim[0]);
      s[1].write(standKnee - kneeLift * kneeSign[0] + trim[1]);
      break;

    case 1: // BL
      s[4].write(standHip + hipMove * hipSign[2] + trim[4]);
      s[5].write(standKnee - kneeLift * kneeSign[2] + trim[5]);
      break;

    case 2: // FR
      s[2].write(standHip - hipMove * hipSign[1] + trim[2]);
      s[3].write(standKnee - kneeLift * kneeSign[1] + trim[3]);
      break;

    case 3: // BR
      s[6].write(standHip - hipMove * hipSign[3] + trim[6]);
      s[7].write(standKnee - kneeLift * kneeSign[3] + trim[7]);
      break;
  }

  gaitPhase = (gaitPhase + 1) & 3;
}

void gaitBackward() {
  unsigned long now = millis();
  if(now - gaitTimer < gaitDelay) return;
  gaitTimer = now;

  int hipMove = 20;
  int kneeLift = 20;

  switch(gaitPhase) {
    case 0: // FL
      s[0].write(standHip - hipMove * hipSign[0] + trim[0]);
      s[1].write(standKnee - kneeLift * kneeSign[0] + trim[1]);
      break;

    case 1: // BL
      s[4].write(standHip - hipMove * hipSign[2] + trim[4]);
      s[5].write(standKnee - kneeLift * kneeSign[2] + trim[5]);
      break;

    case 2: // FR
      s[2].write(standHip + hipMove * hipSign[1] + trim[2]);
      s[3].write(standKnee - kneeLift * kneeSign[1] + trim[3]);
      break;

    case 3: // BR
      s[6].write(standHip + hipMove * hipSign[3] + trim[6]);
      s[7].write(standKnee - kneeLift * kneeSign[3] + trim[7]);
      break;
  }

  gaitPhase = (gaitPhase + 1) & 3;
}
void gaitLeft() {
  unsigned long now = millis();
  if(now - gaitTimer < gaitDelay) return;
  gaitTimer = now;

  int hipTurn = 15;
  int kneeLift = 15;

  switch(gaitPhase) {
    case 0: // FL
      s[0].write(standHip + hipTurn * hipSign[0] + trim[0]);
      s[1].write(standKnee - kneeLift * kneeSign[0] + trim[1]);
      break;

    case 1: // BL
      s[4].write(standHip + hipTurn * hipSign[2] + trim[4]);
      s[5].write(standKnee - kneeLift * kneeSign[2] + trim[5]);
      break;

    case 2: // FR
      s[2].write(standHip - hipTurn * hipSign[1] + trim[2]);
      break;

    case 3: // BR
      s[6].write(standHip - hipTurn * hipSign[3] + trim[6]);
      break;
  }

  gaitPhase = (gaitPhase + 1) & 3;
}
void gaitRight() {
  unsigned long now = millis();
  if(now - gaitTimer < gaitDelay) return;
  gaitTimer = now;

  int hipTurn = 15;
  int kneeLift = 15;

  switch(gaitPhase) {
    case 0: // FR
      s[2].write(standHip + hipTurn * hipSign[1] + trim[2]);
      s[3].write(standKnee - kneeLift * kneeSign[1] + trim[3]);
      break;

    case 1: // BR
      s[6].write(standHip + hipTurn * hipSign[3] + trim[6]);
      s[7].write(standKnee - kneeLift * kneeSign[3] + trim[7]);
      break;

    case 2: // FL
      s[0].write(standHip - hipTurn * hipSign[0] + trim[0]);
      break;

    case 3: // BL
      s[4].write(standHip - hipTurn * hipSign[2] + trim[4]);
      break;
  }

  gaitPhase = (gaitPhase + 1) & 3;
}
