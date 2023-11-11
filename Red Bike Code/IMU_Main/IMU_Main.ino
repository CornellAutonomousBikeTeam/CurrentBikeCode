#include "IMU_Header.h"

IMU imuInstance;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  imuInstance.IMUClasssetup();
}

void loop()
{
  Serial.println("hello");
  float* result = imuInstance.IMUClassloop();
  //this is shooting error "request" for member 'data' in 'result', which is of non-class type "float"
  Serial.println(result.data[0].fval); 
  Serial.println(result.data[1].fval);
  Serial.println(result.data[2].fval);
  //Serial.println(result[0]);
  //Serial.println(result[1]);
  //Serial.println(result[2]);
  Serial.println("after result");
  Serial.println(result[0]);
}
