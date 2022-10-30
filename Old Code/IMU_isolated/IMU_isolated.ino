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
  //byte set_baud[] = {0xF7, 0xE7, 0x00, 0x01};
  //sendMSG(poll_imu_tared_orientation, sizeof(poll_imu_tared_orientation));
  Serial1.write(":1\n"); 
  //sendMSG(poll_imu_tared_orientation, sizeof(poll_imu_tared_orientation));
  Serial.println("Sent message for data");
  while(Serial1.available()){
    if(Serial1.peek() == '\n'){
      //Serial.println("new line");
      Serial1.read();
      //Old way - Serial1.readString();

    }
    else{
      Serial.println(Serial1.read());
      //Old way - Serial.println(Serial1.readString());

    }
  }
  //delay(10);
  // put your main code here, to run repeatedly:
  //float roll_angle = getIMU(0x01, 2);   //get roll angle
  //float roll_rate = getIMU(0x26, 2);    //get roll rate
  //float yaw = getIMU(0x01, 1); //get yaw
  /*Serial.print("Roll angle: ");  Serial.println(roll_angle);
  Serial.print("Roll rate: ");  Serial.println(roll_rate);
  Serial.print("Yaw: ");  Serial.println(yaw);*/

}
