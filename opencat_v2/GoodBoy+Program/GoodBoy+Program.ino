//GoodBoy Robot Dog Code
//Written by Instructables user dentdentarthurdent
//Published 25/05/20

#include <NewPing.h>
#include <Servo.h>

#define trigPin 8
#define echoPin 9
#define trigPin2 10
#define echoPin2 11
#define MAX_DISTANCE 350

NewPing sonar_l(trigPin, echoPin, MAX_DISTANCE);
float duration, distance;
NewPing sonar_r(trigPin2, echoPin2, MAX_DISTANCE);
float duration2, distance2;

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
Servo myservo6;
Servo myservo7;
Servo myservo8;

int pos1 = 65;
int pos2 = 80;
int pos3 = 120;
int pos4 = 80;
int pos5 = 45;
int pos6 = 80;
int pos7 = 125;
int pos8 = 90;

int steps = 0;

int sensorPin = A1;
const int led1 = 12;
const int led2 = 13;
int counter = 0;
int strides;
int milliseconds;

void setup() {
  myservo1.attach(0);
  myservo2.attach(1);
  myservo3.attach(2);
  myservo4.attach(3);
  myservo5.attach(4);
  myservo6.attach(5);
  myservo7.attach(6);
  myservo8.attach(7);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  delay(2000);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//GOODBOY BEHAVIOUR
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void loop() {
  milliseconds = 10000;
  standstill(milliseconds);
  strides = 20;
  stepforward(strides);
  pawtrick();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//STANDSTILL FUNCTION
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int standstill(int x) {
  //NEUTRAL POSITION
  pos1 = 65;
  pos2 = 80;
  pos3 = 120;
  pos4 = 80;
  pos5 = 42;
  pos6 = 95;
  pos7 = 128;
  pos8 = 75;
  myservo1.write(pos1);
  myservo2.write(pos2);
  myservo3.write(pos3);
  myservo4.write(pos4);
  myservo5.write(pos5);
  myservo6.write(pos6);
  myservo7.write(pos7);
  myservo8.write(pos8);
  delay(x);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//PAW TRICK FUNCTION
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int pawtrick() {
  pos1 = 65;
  pos2 = 80;
  pos3 = 120;
  pos4 = 80;
  pos5 = 42;
  pos6 = 80;
  pos7 = 125;
  pos8 = 90;

  myservo1.write(65);
  myservo2.write(80);
  myservo3.write(120);
  myservo4.write(80);
  myservo5.write(45);
  myservo6.write(80);
  myservo7.write(125);
  myservo8.write(90);

  int c = analogRead(sensorPin);

  if (c < 900) {
    digitalWrite(led1, LOW);
    counter = counter + 1;
  }
  else {
    digitalWrite(led1, HIGH);
    counter = 0;
  }
  delay(50);

  if (counter > 50) {
    digitalWrite(led2, HIGH);
    for (pos1 = 65; pos1 <= 85; pos1 += 1) {
      myservo1.write(pos1);
      pos2 = pos2 + 1;
      myservo2.write(pos2);
      delay(25);
    }
    for (pos1 = 85; pos1 <= 110; pos1 += 1) {
      myservo1.write(pos1);
      delay(25);
    }
    for (pos2 = 100; pos2 >= 50; pos2 -= 1) {
      myservo2.write(pos2);
      delay(25);
    }
    for (pos1 = 110; pos1 >= 100; pos1 -= 1) {
      myservo1.write(pos1);
      delay(25);
    }
    delay(3000);
    for (pos1 = 100; pos1 <= 110; pos1 += 1) {
      myservo1.write(pos1);
      delay(25);
    }
    for (pos2 = 50; pos2 <= 100; pos2 += 1) {
      myservo2.write(pos2);
      delay(25);
    }
    for (pos1 = 110; pos1 >= 85; pos1 -= 1) {
      myservo1.write(pos1);
      delay(25);
    }
    for (pos1 = 85; pos1 >= 55; pos1 -= 1) {
      myservo1. write(pos1);
      pos2 = pos2 - 1;
      myservo2.write(pos2);
      delay(25);
    }
    for (pos1 = 55; pos1 <= 65; pos1 += 1) {
      myservo1.write(pos1);
      pos2 = pos2 + 1;
      myservo2.write(pos2);
      delay(25);
    }
    delay(5000);
    digitalWrite(led2, LOW);
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//STRIDES FUNCTION
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

int stepforward(int y) {
  int i = 5;

  //NEUTRAL POSITION
  pos1 = 65;
  pos2 = 80;
  pos3 = 120;
  pos4 = 80;
  pos5 = 42;
  pos6 = 95;
  pos7 = 128;
  pos8 = 75;
  delay(i);

  for (steps = 1; steps <= y; steps += 1) {

    duration = sonar_r.ping();
    distance = (duration / 2) * 0.0343;
    duration2 = sonar_l.ping();
    distance2 = (duration2 / 2) * 0.0343;

    if (distance > 10 && distance2 > 10) {

      //LEFT FRONT STEP FORWARDS MOVEMENT
      for (counter = 1; counter <= 20; counter += 1) {
        pos1 = pos1 + 1;
        myservo1.write(pos1);
        delay(i);
      }
      for (counter = 1; counter <= 14; counter += 1) {
        pos2 = pos2 - 1;
        myservo2.write(pos2);
        delay(i);
      }
      for (counter = 1; counter <= 27; counter += 1) {
        pos1 = pos1 - 1;
        myservo1.write(pos1);
        delay(i);
      }

      //LEFT FRONT STEP FORWARDS (UNDER TEST)
      pos1 = 58;
      pos2 = 66;
      pos3 = 120;
      pos4 = 80;
      pos5 = 42;
      pos6 = 95;
      pos7 = 128;
      pos8 = 75;
      myservo1.write(pos1);
      myservo2.write(pos2);
      myservo3.write(pos3);
      myservo4.write(pos4);
      myservo5.write(pos5);
      myservo6.write(pos6);
      myservo7.write(pos7);
      myservo8.write(pos8);
      delay(300);

      //RIGHT FRONT STEP FORWARDS MOVEMENT
      for (counter = 1; counter <= 20; counter += 1) {
        pos3 = pos3 - 1;
        myservo3.write(pos3);
        delay(i);
      }
      for (counter = 1; counter <= 14; counter += 1) {
        pos4 = pos4 + 1;
        myservo4.write(pos4);
        delay(i);
      }
      for (counter = 1; counter <= 27; counter += 1) {
        pos3 = pos3 + 1;
        myservo3.write(pos3);
        delay(i);
      }

      //RIGHT FRONT STEP FORWARDS (UNDER TEST)
      pos1 = 58;
      pos2 = 66;
      pos3 = 127;
      pos4 = 94;
      pos5 = 42;
      pos6 = 95;
      pos7 = 128;
      pos8 = 75;
      myservo1.write(pos1);
      myservo2.write(pos2);
      myservo3.write(pos3);
      myservo4.write(pos4);
      myservo5.write(pos5);
      myservo6.write(pos6);
      myservo7.write(pos7);
      myservo8.write(pos8);
      delay(300);

      //COG SHIFT MOVEMENT
      for (counter = 1; counter <= 7; counter += 1) {
        pos1 = pos1 + 1;
        pos2 = pos2 + 2;
        pos3 = pos3 - 1;
        pos4 = pos4 - 2;
        pos5 = pos5 - 1;
        pos6 = pos6 + 1;
        pos7 = pos7 + 1;
        pos8 = pos8 - 1;
        myservo1.write(pos1);
        myservo2.write(pos2);
        myservo3.write(pos3);
        myservo4.write(pos4);
        myservo5.write(pos5);
        myservo6.write(pos6);
        myservo7.write(pos7);
        myservo8.write(pos8);
        delay(90);
      }
      for (counter = 1; counter <= 2; counter += 1) {
        pos6 = pos6 + 1;
        pos8 = pos8 - 1;
        myservo6.write(pos6);
        myservo8.write(pos8);
        delay(90);
      }

      //COG SHIFT FROM REAR TO FRONT (UNDER TEST)
      pos1 = 65;
      pos2 = 80;
      pos3 = 120;
      pos4 = 80;
      pos5 = 35;
      pos6 = 104;
      pos7 = 135;
      pos8 = 66;
      myservo1.write(pos1);
      myservo2.write(pos2);
      myservo3.write(pos3);
      myservo4.write(pos4);
      myservo5.write(pos5);
      myservo6.write(pos6);
      myservo7.write(pos7);
      myservo8.write(pos8);
      delay(300);

      //LEFT REAR STEP FORWARDS MOVEMENT
      for (counter = 1; counter <= 10; counter += 1) {
        pos6 = pos6 + 1;
        myservo6.write(pos6);
        delay(i);
      }
      for (counter = 1; counter <= 30; counter += 1) {
        pos5 = pos5 + 1;
        myservo5.write(pos5);
        delay(i);
      }
      for (counter = 1; counter <= 19; counter += 1) {
        pos6 = pos6 - 1;
        myservo6.write(pos6);
        delay(i);
      }
      for (counter = 1; counter <= 28; counter += 1) {
        pos5 = pos5 - 1;
        myservo5.write(pos5);
        delay(i);
      }
      for (counter = 1; counter <= 5; counter += 1) {
        pos5 = pos5 + 1;
        myservo5.write(pos5);
        delay(i);
      }

      //LEFT REAR STEP FORWARDS
      pos1 = 65;
      pos2 = 80;
      pos3 = 120;
      pos4 = 80;
      pos5 = 42;
      pos6 = 95;
      pos7 = 135;
      pos8 = 66;
      myservo1.write(pos1);
      myservo2.write(pos2);
      myservo3.write(pos3);
      myservo4.write(pos4);
      myservo5.write(pos5);
      myservo6.write(pos6);
      myservo7.write(pos7);
      myservo8.write(pos8);
      delay(300);

      //RIGHT REAR STEP FORWARDS
      for (counter = 1; counter <= 10; counter += 1) {
        pos8 = pos8 - 1;
        myservo8.write(pos8);
        delay(i);
      }
      for (counter = 1; counter <= 30; counter += 1) {
        pos7 = pos7 - 1;
        myservo7.write(pos7);
        delay(i);
      }
      for (counter = 1; counter <= 19; counter += 1) {
        pos8 = pos8 + 1;
        myservo8.write(pos8);
        delay(i);
      }
      for (counter = 1; counter <= 28; counter += 1) {
        pos7 = pos7 + 1;
        myservo7.write(pos7);
        delay(i);
      }
      for (counter = 1; counter <= 5; counter += 1) {
        pos7 = pos7 - 1;
        myservo7.write(pos7);
        delay(i);
      }

      //NEUTRAL POSITION
      pos1 = 65;
      pos2 = 80;
      pos3 = 120;
      pos4 = 80;
      pos5 = 42;
      pos6 = 95;
      pos7 = 128;
      pos8 = 75;
      myservo1.write(pos1);
      myservo2.write(pos2);
      myservo3.write(pos3);
      myservo4.write(pos4);
      myservo5.write(pos5);
      myservo6.write(pos6);
      myservo7.write(pos7);
      myservo8.write(pos8);
      delay(300);
    }
    else {
      delay(500);
    }
  }
}
