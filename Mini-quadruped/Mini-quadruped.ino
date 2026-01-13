/**************************************************
  MINI QUADRUPED (2DOF) - SINGLE FILE FIRMWARE
  Stand Mode: LOW
  Added: case 'S' -> standLow()
**************************************************/

#include <Wire.h>
#include <EEPROM.h>
#include <IRremote.h>
#include <MPU6050.h>
#include <Servo.h>

#define ENABLE_BT        1
#define ENABLE_IR        1
#define ENABLE_IMU       1
#define ENABLE_EEPROM    1
#define ENABLE_SMOOTHING 1

enum {
  LF_HIP  = 2,
  LF_KNEE = 3,
  RF_HIP  = 4,
  RF_KNEE = 5,
  LB_HIP  = 6,
  LB_KNEE = 7,
  RB_HIP  = 8,
  RB_KNEE = 9,
};

#define NUM_SERVOS 10

int servoPin[NUM_SERVOS] = {
  -1, -1,
   6,  //LF hip
   9,  //LF knee
   3,  //RF hip
   5,  //RF knee
   8,  //LB hip
  13,  //LB knee
  10,  //RB hip
  11   //RB knee
};

#define IR_PIN 12
#define BT_RX 0
#define BT_TX 1

#define MPU_SDA A4
#define MPU_SCL A5

Servo sv[NUM_SERVOS];
MPU6050 mpu;

int16_t offset[NUM_SERVOS];

float pitch = 0;
float Kp = 0.25;

char cmd = 'S'; // default STAND
unsigned long gaitTimer = 0;
int gaitPhase = 0;

int apply(int idx, int angle){
  angle += offset[idx];
  angle = constrain(angle, 60, 120);
  return angle;
}

void writeServo(int idx, int angle){
  if(idx < NUM_SERVOS && servoPin[idx] >= 0){
    sv[idx].write(angle);
  }
}

void standLow(){
  writeServo(LF_HIP,  90);
  writeServo(RF_HIP,  90);
  writeServo(LB_HIP,  90);
  writeServo(RB_HIP,  90);

  writeServo(LF_KNEE, 105);
  writeServo(RF_KNEE, 105);
  writeServo(LB_KNEE, 105);
  writeServo(RB_KNEE, 105);
}

void trotForward(){
  if(gaitPhase == 0){
    writeServo(LF_HIP, 110);
    writeServo(RB_HIP, 110);
    writeServo(RF_HIP,  70);
    writeServo(LB_HIP,  70);
  } else {
    writeServo(LF_HIP,  70);
    writeServo(RB_HIP,  70);
    writeServo(RF_HIP, 110);
    writeServo(LB_HIP, 110);
  }
}

void trotReverse(){
  int a = 10;
  if(gaitPhase == 0){
    writeServo(LF_HIP, 90-a);
    writeServo(RB_HIP, 90-a);
    writeServo(RF_HIP, 90+a);
    writeServo(LB_HIP, 90+a);
  } else {
    writeServo(LF_HIP, 90+a);
    writeServo(RB_HIP, 90+a);
    writeServo(RF_HIP, 90-a);
    writeServo(LB_HIP, 90-a);
  }
}

void tankLeft(){
  writeServo(LF_HIP, 110);
  writeServo(LB_HIP, 110);
  writeServo(RF_HIP,  70);
  writeServo(RB_HIP,  70);
}

void tankRight(){
  writeServo(LF_HIP,  70);
  writeServo(LB_HIP,  70);
  writeServo(RF_HIP, 110);
  writeServo(RB_HIP, 110);
}

void imuUpdate(){
#if ENABLE_IMU
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  pitch = gx/131.0 * 0.01;
  int kneeAdj = pitch * Kp;
  writeServo(LF_KNEE, 105 + kneeAdj);
  writeServo(RF_KNEE, 105 + kneeAdj);
  writeServo(LB_KNEE, 105 + kneeAdj);
  writeServo(RB_KNEE, 105 + kneeAdj);
#endif
}

void handleCmd(char c){
  cmd = c;
}

void setup(){
  Serial.begin(115200);
  IrReceiver.begin(IR_PIN);

  Wire.begin();
  mpu.initialize();

  for(int i=0;i<NUM_SERVOS;i++){
    if(servoPin[i]>=0){
      sv[i].attach(servoPin[i]);
      sv[i].write(90);
    }
  }
  standLow();
}

void loop(){

#if ENABLE_IR
  if(IrReceiver.decode()){
    handleCmd('S'); // IR triggers stand for now
    IrReceiver.resume();
  }
#endif

#if ENABLE_BT
  if(Serial.available()){
    handleCmd(Serial.read());
  }
#endif

if(millis()-gaitTimer > 120){
  gaitTimer = millis();
  gaitPhase = 1-gaitPhase;

  switch(cmd){
    case 'F': trotForward(); break;
    case 'B': trotReverse(); break;
    case 'L': tankLeft(); break;
    case 'R': tankRight(); break;
    case 'S': standLow(); break;   // <--- added as requested
    default: standLow(); break;
  }
}

imuUpdate();
}
