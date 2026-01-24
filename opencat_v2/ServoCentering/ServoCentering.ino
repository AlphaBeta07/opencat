#include <Servo.h>

Servo FL_Hip;
Servo FL_Knee;
Servo FR_Hip;
Servo FR_Knee;
Servo BL_Hip;
Servo BL_Knee;
Servo BR_Hip;
Servo BR_Knee;

void setup() {
  FL_Hip.attach(2);
  FL_Knee.attach(3);
  FR_Hip.attach(4);
  FR_Knee.attach(5);
  BL_Hip.attach(6);
  BL_Knee.attach(7);
  BR_Hip.attach(8);
  BR_Knee.attach(9);

  setAll(90);
}

void loop() {}

void setAll(int a) {
  FL_Hip.write(a);
  FL_Knee.write(a);
  FR_Hip.write(a);
  FR_Knee.write(a);
  BL_Hip.write(a);
  BL_Knee.write(a);
  BR_Hip.write(a);
  BR_Knee.write(a);
}
