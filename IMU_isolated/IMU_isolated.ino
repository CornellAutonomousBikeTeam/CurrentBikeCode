#include "IMU_Serial.h"

void setup() {
  initIMU();
  Serial.begin(19200); //Testing purposes - Serial Monitor
}

struct roll_t {
  float roll_angle;
  float roll_rate;
  float yaw;
};

//struct that contains roll angle, roll rate, yaw
roll_t imu_data;
//array that contains euler angles in pitch, yaw, roll order
float euler_angles[3];
//array that contains the corrected gyro rate in radians/sec
float gyro_rate[3];

//Main loop
void loop() {
  //Send the "Get tared orientation as euler angles" command
  Serial1.write(":1\n");
  Serial.print("Tared orientation: ");
  readBuffer("euler");
  imu_data.roll_angle = euler_angles[2];
  imu_data.yaw = euler_angles[1];
  Serial1.write(":38\n");
  Serial.print("Gyro rate: ");
  readBuffer("gyro");
  imu_data.roll_rate = gyro_rate[2];
  Serial.println("Roll angle: ");
  Serial.println(imu_data.roll_angle, 10);
  //Serial.print(euler_angles[2], 10);
  Serial.println("Roll rate: ");
  Serial.println(imu_data.roll_rate, 10);
  //Serial.print(gyro_rate[2], 10);
  Serial.println("Yaw: ");
  Serial.println(imu_data.yaw, 10);
  //Serial.print(euler_angles[1], 10);*/
}

roll_t readBuffer(String command) {
  int i = 0;
  String data;
  while(Serial1.available()){
    if(Serial1.peek() == '\n'){
      if (i == 2 && command.equals("euler")) {
        euler_angles[i] = data.toFloat();
      }
      else if (i == 2) gyro_rate[i] = data.toFloat();
      Serial1.read();
      Serial.println();
    }
    else{
      char ch = Serial1.read();
      if (ch == ',') {
          if (command.equals("euler")) euler_angles[i] = data.toFloat();
          else gyro_rate[i] = data.toFloat();
          //Serial.println(data);
          data = "";
          i++;
      }
      else {
        data += ch;
      }
      Serial.print(ch);
    }
  }
  if (command.equals("euler")) {
    return
  }
}
