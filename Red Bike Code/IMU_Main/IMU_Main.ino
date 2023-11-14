#include "IMU_Header.h"



void setup()
{
  IMU imuInstance;
  Serial.begin(9600);
  while (!Serial)
    ;
  imuInstance.IMUClasssetup();
}

void loop()
{
  IMU imuInstance;
  //Serial.println("hello");
  float* result = imuInstance.IMUClassloop();
  Serial.println(imuInstance.data[0].fval); 
  Serial.println(imuInstance.data[1].fval);
  Serial.println(imuInstance.data[2].fval);
  //Serial.println(result[0]);
  //Serial.println(result[1]);
  //Serial.println(result[2]);
  Serial.println("after result");
}
