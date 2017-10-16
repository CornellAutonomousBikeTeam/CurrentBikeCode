#include "IMU_Serial.h"

void setup() {
  // put your setup code here, to run once:
  initIMU();
  Serial.begin(19200);
}

struct roll_t {
  float roll_angle;
  float roll_rate;
  float yaw;
};

roll_t roll_data;
float euler_angles[3];
float gyro_rate[3];

void loop() {
  Serial1.write(":1\n");
  Serial.print("Tared orientation: ");
  readBuffer("euler");
  roll_data.roll_angle = euler_angles[2];
  roll_data.yaw = euler_angles[1];
  Serial1.write(":38\n");
  Serial.print("Gyro rate: ");
  readBuffer("gyro");
  roll_data.roll_rate = gyro_rate[2];
  Serial.println("Roll angle: ");
  Serial.println(roll_data.roll_angle, 10);
  //Serial.print(euler_angles[2], 10);
  Serial.println("Roll rate: ");
  Serial.println(roll_data.roll_rate, 10);
  //Serial.print(gyro_rate[2], 10);
  Serial.println("Yaw: ");
  Serial.println(roll_data.yaw, 10);
  //Serial.print(euler_angles[1], 10);*/
}

void readBuffer(String command) {
  int i = 0;
  String data;
  while(Serial1.available()){
    if(Serial1.peek() == '\n'){
      Serial1.read();
      Serial.println();
    }
    else{
      char ch = Serial1.read();
      if (String(ch).equals(",")) {
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
}
