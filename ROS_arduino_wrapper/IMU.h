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

struct roll_t {
  float roll_angle;
  float roll_rate;
  float yaw; 
};
extern roll_t imu_data;
extern float euler_angles[3]; //array that contains euler angles in pitch, yaw, roll order
extern float gyro_rate[3]; //array that contains the corrected gyro rate in radians/sec


/*Define functions*/
//Initialize IMU
void initIMU(void);

//IMU data
float getIMU(byte, int);

struct roll_t updateIMUData();

void readBuffer(float*);

void updateIMUSerial();

#endif //IMU_h

