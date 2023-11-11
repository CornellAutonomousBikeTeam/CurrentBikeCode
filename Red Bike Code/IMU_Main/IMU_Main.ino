#include "IMU_Header.h"
void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Hello before init");
  IMU imuInstance;
  imuInstance.IMUClasssetup();
  Serial.println("Hello after init");
}

void loop()
{
  IMU imuInstance;
  Serial.println("hello");
  float *result = imuInstance.IMUClassloop();
  Serial.println(result.data[0].fval);
  Serial.println(result.data[1].fval);
  Serial.println(result.data[2].fval);
  Serial.println("after result");
  Serial.println(result[0]);
  // Do something with result
}
