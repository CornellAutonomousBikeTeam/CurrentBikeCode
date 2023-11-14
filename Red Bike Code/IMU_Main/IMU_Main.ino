#include "IMU_Header.h"

void setup()
{
  IMU imuInstance;
  Serial.begin(9600);
  while (!Serial)
    ;
  imuInstance.IMUClasssetup(); // Setting up the IMUInstance
}

void loop() // Constantly prints out the change in values of each axis.
{
  IMU imuInstance;
  float *result = imuInstance.IMUClassloop();
  Serial.println(imuInstance.data[0].fval); // Yanjun & Alex: this is our current implementation to print out the
  Serial.println(imuInstance.data[1].fval); // values from the IMU. It is not ideal since we are directly accessing the internal variables
  Serial.println(imuInstance.data[2].fval); // of the imuInstance, but it is the only way that we can get it working for now.
}
