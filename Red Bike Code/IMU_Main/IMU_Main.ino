#include "IMU_Header.h"

void setup() {
  IMU imuInstance;
  imuInstance.IMUClasssetup();
}

void loop() {
  IMU imuInstance;
  float* result = imuInstance.IMUClassloop();
  // Do something with result
}
