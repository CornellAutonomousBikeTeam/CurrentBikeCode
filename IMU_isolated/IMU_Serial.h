#ifndef IMU_Serial_h
#define IMU_Serial_h

#include <SPI.h>
#include <math.h>


/*Define functions*/
//Initialize IMU
void initIMU(void);

//IMU data
float getIMU(byte, int);

#endif
