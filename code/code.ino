/****************************************************
  Leo the Pet Cat — Enhanced Version (Nano DMP)
  - HC05 Bluetooth control
  - Commands: F,B,L,R,S,A,M,D,C
  - Postures: Stand, Sit, Lay, Sleep
  - Stop = Freeze
  - Walk Gait
  - Light Stabilization (DMP Pitch/Roll)
  - Arduino Nano / SG90 / No Debug Prints
****************************************************/
// https://chatgpt.com/share/69665a56-306c-8009-91d1-57aafb79f5ba

#include <Wire.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/************** MPU6050 **************/
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
volatile bool mpuInterrupt = false;
float pitch = 0, roll = 0;

/************** SERVOS **************/
Servo fl_hip, fl_knee;   // front-left
Servo fr_hip, fr_knee;   // front-right
Servo bl_hip, bl_knee;   // back-left
Servo br_hip, br_knee;   // back-right

/************** SERVO ANGLES **************/
int fl_hip_angle = 90;
int fl_knee_angle = 90;
int fr_hip_angle = 90;
int fr_knee_angle = 90;
int bl_hip_angle = 90;
int bl_knee_angle = 90;
int br_hip_angle = 90;
int br_knee_angle = 90;

/************** STATES **************/
enum MotionState {
  STATE_STAND,
  STATE_SIT,
  STATE_LAY,
  STATE_SLEEP,
  STATE_WALK_FWD,
  STATE_WALK_BACK,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT,
  STATE_STOP
};

MotionState currentState = STATE_STAND;
MotionState prevWalkingState = STATE_STAND;
char btCmd = 0;

/************** GAIT **************/
unsigned long gaitTimer = 0;
int gaitStep = 0;
const int gaitDelay = 90;
const int turnDelay = 110;

/************** STABILIZATION **************/
bool stabilizationEnabled = true;
float stabGain = 4.0;

/************** FUNCTION DECLARES **************/
void applyAngles();
void smoothMove(int &cur, int target, int step = 2);
void standPosture(bool first = false);
void sitPosture(bool first = false);
void layPosture(bool first = false);
void sleepPosture(bool first = false);
void doWalkForward();
void doWalkBackward();
void doTurnLeft();
void doTurnRight();
void doStopFreeze();
void commandParser();
void gaitEngine();
void mpuStabilize();

/************** INTERRUPT **************/
void dmpDataReady() {
  mpuInterrupt = true;
}

/************** DMP INIT **************/
void setupMPU() {
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Offsets (example defaults for SG90 cat)
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

/************** SETUP **************/
void setup() {
  // Servos (Nano pins)
  fl_hip.attach(2); fl_knee.attach(3);
  fr_hip.attach(4); fr_knee.attach(5);
  bl_hip.attach(6); bl_knee.attach(7);
  br_hip.attach(8); br_knee.attach(9);

  // HC05 on hardware Serial
  Serial.begin(9600);

  setupMPU();

  standPosture(true); // initial
}
/************** APPLY SERVO ANGLES **************/
void applyAngles() {
  fl_hip.write(fl_hip_angle);
  fl_knee.write(fl_knee_angle);
  fr_hip.write(fr_hip_angle);
  fr_knee.write(fr_knee_angle);
  bl_hip.write(bl_hip_angle);
  bl_knee.write(bl_knee_angle);
  br_hip.write(br_hip_angle);
  br_knee.write(br_knee_angle);
}

/************** SMOOTH SERVO MOVE **************/
void smoothMove(int &cur, int target, int step) {
  if (cur < target) cur += step;
  else if (cur > target) cur -= step;
}

/************** POSTURES **************/
void standPosture(bool first) {
  // neutral standing
  int fh = 85, fk = 105;
  int rh = 95, rk = 75;

  if (first) {
    fl_hip_angle = fh; fl_knee_angle = fk;
    fr_hip_angle = fh; fr_knee_angle = fk;
    bl_hip_angle = rh; bl_knee_angle = rk;
    br_hip_angle = rh; br_knee_angle = rk;
    applyAngles();
  } else {
    smoothMove(fl_hip_angle, fh);
    smoothMove(fl_knee_angle, fk);
    smoothMove(fr_hip_angle, fh);
    smoothMove(fr_knee_angle, fk);
    smoothMove(bl_hip_angle, rh);
    smoothMove(bl_knee_angle, rk);
    smoothMove(br_hip_angle, rh);
    smoothMove(br_knee_angle, rk);
  }
}

/************** SIT (rear fold) **************/
void sitPosture(bool first) {
  // rear legs folded, front semi-up
  int fh = 85, fk = 105;
  int rh = 125, rk = 45;

  if (first) {
    fl_hip_angle = fh; fl_knee_angle = fk;
    fr_hip_angle = fh; fr_knee_angle = fk;
    bl_hip_angle = rh; bl_knee_angle = rk;
    br_hip_angle = rh; br_knee_angle = rk;
    applyAngles();
  } else {
    smoothMove(fl_hip_angle, fh);
    smoothMove(fl_knee_angle, fk);
    smoothMove(fr_hip_angle, fh);
    smoothMove(fr_knee_angle, fk);
    smoothMove(bl_hip_angle, rh);
    smoothMove(bl_knee_angle, rk);
    smoothMove(br_hip_angle, rh);
    smoothMove(br_knee_angle, rk);
  }
}

/************** LAY (full collapse) **************/
void layPosture(bool first) {
  int fh = 115, fk = 65;
  int rh = 115, rk = 55;

  if (first) {
    fl_hip_angle = fh; fl_knee_angle = fk;
    fr_hip_angle = fh; fr_knee_angle = fk;
    bl_hip_angle = rh; bl_knee_angle = rk;
    br_hip_angle = rh; br_knee_angle = rk;
    applyAngles();
  } else {
    smoothMove(fl_hip_angle, fh);
    smoothMove(fl_knee_angle, fk);
    smoothMove(fr_hip_angle, fh);
    smoothMove(fr_knee_angle, fk);
    smoothMove(bl_hip_angle, rh);
    smoothMove(bl_knee_angle, rk);
    smoothMove(br_hip_angle, rh);
    smoothMove(br_knee_angle, rk);
  }
}

/************** SLEEP (relaxed lay) **************/
void sleepPosture(bool first) {
  int fh = 120, fk = 70;
  int rh = 120, rk = 60;

  if (first) {
    fl_hip_angle = fh; fl_knee_angle = fk;
    fr_hip_angle = fh; fr_knee_angle = fk;
    bl_hip_angle = rh; bl_knee_angle = rk;
    br_hip_angle = rh; br_knee_angle = rk;
    applyAngles();
  } else {
    smoothMove(fl_hip_angle, fh);
    smoothMove(fl_knee_angle, fk);
    smoothMove(fr_hip_angle, fh);
    smoothMove(fr_knee_angle, fk);
    smoothMove(bl_hip_angle, rh);
    smoothMove(bl_knee_angle, rk);
    smoothMove(br_hip_angle, rh);
    smoothMove(br_knee_angle, rk);
  }
}

/************** STOP FREEZE **************/
void doStopFreeze() {
  // No movement, just keep last angles
  applyAngles();
}

/************** COMMAND PARSER (HC05) **************/
void commandParser() {
  if (Serial.available()) {
    btCmd = Serial.read();

    switch (btCmd) {
      case 'A': currentState = STATE_STAND; break;
      case 'M': currentState = STATE_SIT; break;
      case 'D': currentState = STATE_LAY; break;
      case 'C': currentState = STATE_SLEEP; break;
      case 'F': prevWalkingState = STATE_WALK_FWD; currentState = STATE_WALK_FWD; break;
      case 'B': prevWalkingState = STATE_WALK_BACK; currentState = STATE_WALK_BACK; break;
      case 'L': prevWalkingState = STATE_TURN_LEFT; currentState = STATE_TURN_LEFT; break;
      case 'R': prevWalkingState = STATE_TURN_RIGHT; currentState = STATE_TURN_RIGHT; break;
      case 'S': currentState = STATE_STOP; break;
    }
  }
}
/************** GAIT ENGINE **************/
void gaitEngine() {
  if (millis() - gaitTimer < gaitDelay) return;
  gaitTimer = millis();

  switch (currentState) {
    case STATE_WALK_FWD:    doWalkForward(); break;
    case STATE_WALK_BACK:   doWalkBackward(); break;
    case STATE_TURN_LEFT:   doTurnLeft(); break;
    case STATE_TURN_RIGHT:  doTurnRight(); break;
    default: break;
  }
  applyAngles();
}

/************** WALK FORWARD GAIT **************/
void doWalkForward() {
  switch (gaitStep) {
    case 0:
      fl_knee_angle = 120; fr_knee_angle = 80;
      bl_knee_angle = 70; br_knee_angle = 110;
      break;
    case 1:
      fl_knee_angle = 105; fr_knee_angle = 95;
      bl_knee_angle = 90; br_knee_angle = 90;
      break;
    case 2:
      fl_knee_angle = 80; fr_knee_angle = 120;
      bl_knee_angle = 110; br_knee_angle = 70;
      break;
    case 3:
      fl_knee_angle = 95; fr_knee_angle = 105;
      bl_knee_angle = 90; br_knee_angle = 90;
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

/************** WALK BACKWARD **************/
void doWalkBackward() {
  switch (gaitStep) {
    case 0:
      fl_knee_angle = 80; fr_knee_angle = 120;
      bl_knee_angle = 110; br_knee_angle = 70;
      break;
    case 1:
      fl_knee_angle = 95; fr_knee_angle = 105;
      bl_knee_angle = 90; br_knee_angle = 90;
      break;
    case 2:
      fl_knee_angle = 120; fr_knee_angle = 80;
      bl_knee_angle = 70; br_knee_angle = 110;
      break;
    case 3:
      fl_knee_angle = 105; fr_knee_angle = 95;
      bl_knee_angle = 90; br_knee_angle = 90;
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

/************** TURN LEFT **************/
void doTurnLeft() {
  switch (gaitStep) {
    case 0:
      fl_knee_angle = 120; fr_knee_angle = 120;
      bl_knee_angle = 80; br_knee_angle = 80;
      break;
    case 1:
      fl_knee_angle = 105; fr_knee_angle = 105;
      bl_knee_angle = 95; br_knee_angle = 95;
      break;
    case 2:
      fl_knee_angle = 80; fr_knee_angle = 80;
      bl_knee_angle = 120; br_knee_angle = 120;
      break;
    case 3:
      fl_knee_angle = 95; fr_knee_angle = 95;
      bl_knee_angle = 105; br_knee_angle = 105;
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

/************** TURN RIGHT **************/
void doTurnRight() {
  switch (gaitStep) {
    case 0:
      fl_knee_angle = 80; fr_knee_angle = 80;
      bl_knee_angle = 120; br_knee_angle = 120;
      break;
    case 1:
      fl_knee_angle = 95; fr_knee_angle = 95;
      bl_knee_angle = 105; br_knee_angle = 105;
      break;
    case 2:
      fl_knee_angle = 120; fr_knee_angle = 120;
      bl_knee_angle = 80; br_knee_angle = 80;
      break;
    case 3:
      fl_knee_angle = 105; fr_knee_angle = 105;
      bl_knee_angle = 95; br_knee_angle = 95;
      break;
  }
  gaitStep = (gaitStep + 1) % 4;
}

/************** MPU STABILIZATION **************/
void mpuStabilize() {
  if (!stabilizationEnabled) return;
  if (!dmpReady) return;
  if (!mpuInterrupt) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
    return;
  }
  if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    pitch = ypr[1] * 180 / M_PI;
    roll  = ypr[2] * 180 / M_PI;

    // Light stabilization adjustments
    fl_hip_angle -= roll / stabGain;
    fr_hip_angle -= roll / stabGain;
    bl_hip_angle += roll / stabGain;
    br_hip_angle += roll / stabGain;

    fl_knee_angle += pitch / stabGain;
    fr_knee_angle += pitch / stabGain;
    bl_knee_angle -= pitch / stabGain;
    br_knee_angle -= pitch / stabGain;
  }
}
/************** MAIN LOOP **************/
void loop() {

  // Check HC05 commands
  commandParser();

  // Stabilize (DMP Mode)
  mpuStabilize();

  // State Machine
  switch (currentState) {

    case STATE_STAND:
      standPosture(false);
      break;

    case STATE_SIT:
      sitPosture(false);
      break;

    case STATE_LAY:
      layPosture(false);
      break;

    case STATE_SLEEP:
      sleepPosture(false);
      break;

    case STATE_WALK_FWD:
    case STATE_WALK_BACK:
    case STATE_TURN_LEFT:
    case STATE_TURN_RIGHT:
      gaitEngine();
      break;

    case STATE_STOP:
      doStopFreeze();
      break;
  }

  applyAngles();
}

/************** END OF FILE **************/
