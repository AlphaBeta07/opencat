#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50

// Only the 8 servos you use
int servoChannels[8] = {0, 1, 4, 5, 8, 9, 12, 13};
int currentServo = 0;

// Center pulse storage
int servoCenter[8] = {
  1500, 1625,      // FL Hip 0, FL Knee 1
  1500, 1500,      // FR Hip 2, FR Knee 3
  1500, 1445,      // BR Hip 4, BR Knee 5
  1500, 1500       // BL Hip 6, BL Knee 7
};

void setup() {
  Serial.begin(9600);

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Apply initial centers
  for (int i = 0; i < 8; i++) {
    pwm.writeMicroseconds(servoChannels[i], servoCenter[i]);
  }

  Serial.println("=== SERVO CENTER CALIBRATION MODE ===");
  Serial.println("N: Next  | P: Previous");
  Serial.println("+: Move + | -: Move -");
  Serial.println("S: Save & Exit");
  printStatus();
}

void loop() {
  if (!Serial.available()) return;

  char cmd = Serial.read();

  if (cmd == 'N') {
    currentServo = (currentServo + 1) % 8;
    printStatus();
  }

  if (cmd == 'P') {
    currentServo = (currentServo + 7) % 8;
    printStatus();
  }

  if (cmd == '+') {
    servoCenter[currentServo] += 5;
    updateServo();
  }

  if (cmd == '-') {
    servoCenter[currentServo] -= 5;
    updateServo();
  }

  if (cmd == 'S') {
    printCenters();
    while (1); // stop here
  }
}

void updateServo() {
  servoCenter[currentServo] = constrain(servoCenter[currentServo], 500, 2500);
  pwm.writeMicroseconds(servoChannels[currentServo], servoCenter[currentServo]);
  printStatus();
}

void printStatus() {
  Serial.print("Servo ");
  Serial.print(currentServo);
  Serial.print(" (CH ");
  Serial.print(servoChannels[currentServo]);
  Serial.print(") = ");
  Serial.println(servoCenter[currentServo]);
}

void printCenters() {
  Serial.println("\n=== COPY THESE VALUES ===");
  Serial.print("int servoCenter[16] = {");

  for (int i = 0; i < 16; i++) {
    bool found = false;
    for (int j = 0; j < 8; j++) {
      if (servoChannels[j] == i) {
        Serial.print(servoCenter[j]);
        found = true;
      }
    }
    if (!found) Serial.print("0");
    if (i < 15) Serial.print(", ");
  }

  Serial.println("};");
}
