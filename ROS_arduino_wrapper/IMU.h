#ifndef IMU_h
#define IMU_h

#include <SPI.h>
#include <math.h>

/*Define definite variables*/
//IMU
#define CSN 65
#define SO 74
#define SI 75
#define CLK 76

/*Define functions*/
//Initialize IMU
void initIMU(void);

//New retrieve IMU data
void readBuffer(float[]);
void updateIMUDataSerial(void);

//Old retrieve IMU data
//float getIMU(byte, int);


#endif //IMU_h


