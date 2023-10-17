#include "IMU_Header.h"
#include "IMU_Class.cpp"
void setup() {
  IMU imuInstance;
  imuInstance.IMUClasssetup();
}

void loop() {
  IMU imuInstance;
  float* result = imuInstance.IMUClassloop();
  // Do something with result
}
