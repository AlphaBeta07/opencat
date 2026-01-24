#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  Serial.println("MPU Test");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  float pitch = atan2(ax, sqrt(ay*ay + az*az)) * 57.3;
  float roll  = atan2(ay, sqrt(ax*ax + az*az)) * 57.3;

  float pitch_corrected = -pitch; // arrow backward
  float roll_corrected = roll;

  Serial.print("Pitch_raw: ");
  Serial.print(pitch);
  Serial.print(" Pitch_corr: ");
  Serial.print(pitch_corrected);
  Serial.print(" Roll: ");
  Serial.println(roll_corrected);
  
  delay(1000);
}
