#include "IMU_Header.h"

void setup() {
  IMU imuInstance;
  imuInstance.IMUClasssetup();
}

void loop() {
  IMU imuInstance;
  float* result = imuInstance.IMUClassloop();
  Serial.println(result[0]);
  // Do something with result
}
