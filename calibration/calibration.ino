#include <Servo.h>
Servo s[8];
void setup() {
  int pins[8] = {2,3,4,5,6,7,8,9};
  for(int i=0;i<8;i++){
    s[i].attach(pins[i]);
    s[i].write(90);
  }
}
void loop() {}
  