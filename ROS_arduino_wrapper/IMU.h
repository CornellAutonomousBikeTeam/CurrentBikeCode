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

extern roll_t imu_data; //Holds data that IMU returns
extern float euler_angles[3]; //array that contains euler angles in pitch, yaw, roll order
extern float gyro_rate[3]; //array that contains the corrected gyro rate in radians/sec


/* Initialize the IMU using SPI */
void initIMU(void);

/* Sends command (taken as param) to the imu as well as the location in the relevant data array
 * and returns data accordingly
*/
float getIMU(byte, int);

/* Retrieve data from IMU about roll angle and rate and return it */
struct roll_t updateIMUData();

/* Not in use, called by updateIMUSerial() */
void readBuffer(float*);

/* Not in use, used in attempt to switch the bike to the serial protocal from SPI*/
void updateIMUSerial();

#endif //IMU_h

