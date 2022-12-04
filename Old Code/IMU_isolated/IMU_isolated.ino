#include "IMU_Serial.h"
void setup() {
  // put your setup code here, to run once:
  initIMU();
  Serial.begin(9600);
  //Serial1.write(":232\n"); 
}

//  method for sending hex messages to the gps
void sendMSG(byte *msg, byte msgLength) {
  for (int i = 0; i < msgLength; i++) {
    Serial1.write(msg[i]);
  }
}

struct roll_t {
  float rate;
  float angle;
  float yaw; 
};

// Retrieve data from IMU about roll angle and rate and return it
struct roll_t updateIMUData() {
  roll_t roll_data;

  //get data from IMU
  float roll_angle = getIMU(0x01, 2);   //get roll angle
  float roll_rate = getIMU(0x26, 2);    //get roll rate
  float yaw = getIMU(0x01, 1); //get yaw
  roll_data.angle = roll_angle;
  roll_data.rate = roll_rate;
  roll_data.yaw = yaw;
  return roll_data;
}

void loop() {
  byte poll_imu_tared_orientation[] = {0xF7, 0x01, 0x00, 0x01};
  Serial1.write(":1\n"); 
  while(Serial1.available()){
    Serial.println(Serial1.read());
  }
}
