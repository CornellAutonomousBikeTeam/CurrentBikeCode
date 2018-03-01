#include "IMU.h"


/*Variables*/
roll_t imu_data;
float euler_angles[3]; //array that contains euler angles in pitch, yaw, roll order
float gyro_rate[3]; //array that contains the corrected gyro rate in radians/sec

/*IMU Setup and Functions*/
/////////////////////////////////////////////////////////////////////////////////////
SPISettings settings(6000000, MSBFIRST, SPI_MODE0 ); //variable to hold SPI settings this is only 6 times per second

union u_data {
    byte b[4];
    float fval;
} data[3]; // Create 3 unions, one for each value from the IMU
           // Euler angle: x-axis (pitch), y-axis (yaw), z-axis (roll)
           // Gyroscope: x-axis (pitch rate), y-axis (yaw rate), z-axis (roll rate)

//function to transfer commands through SPI
byte transferByte(byte byteToWrite) {     
  byte Result = 0x00;
  digitalWrite(CSN,LOW);
  Result = SPI.transfer(byteToWrite);
  digitalWrite(CSN,HIGH); 
  return Result; 
}

//function to swap endian
void endianSwap(byte temp[4]) {
  byte myTemp = temp[0];
  temp[0] = temp[3];
  temp[3] = myTemp;
  myTemp = temp[1];
  temp[1] = temp[2];
  temp[2] = myTemp;
}
/////////////////////////////////////////////////////////////////////////////////////

/*Initialize IMU*/
/////////////////////////////////////////////////////////////////////////////////////
void initIMU(void){
  //IMU
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN, LOW);
  //Initialize SPI
  SPI.begin();
  SPI.beginTransaction(settings);
  //Initialize serial
  Serial1.begin(115200);
}
/*
  SPI.transfer(0x50); //set streaming slots
  delay(1);
  SPI.transfer(0x01); //get tared euler angles command
  SPI.transfer(0xFF); //don't care
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0x52);
  delay(1);
*/
//  //Check
//  unsigned int Result = SPI.transfer(0x51); //get streaming slots
//  Serial.println(Result);
//  
////
////  SPI.transfer(0x52); //set streaming timing
////  SPI.transfer(10000); //streaming interval ÂµS
////  SPI.transfer(0xFFFFFFFF, 4); //streaming duration
////  SPI.transfer(500);//Delay ÂµS
//
//  //check timing 
//  unsigned int Result2 = SPI.transfer(0x53);
//  Serial.println(Result2);
//
//  SPI.transfer(0x55);//start streaming
//  
//  return;
//}
///////////////////////////////////////////////////////////////////////////////////////

/*getIMU*/
/////////////////////////////////////////////////////////////////////////////////////
//float getIMU(byte commandToWrite){
//  for (int ii=0; ii<3; ii++) {
//    for (int jj=0; jj<4; jj++) {
//      data[ii].b[jj] =  SPI_RDR; //read and store data directly from spi received data register
//    }
// } 
//  byte data = B00000000;
//  for (int i=0; i<8; i++) {
//   byte bit0 = digitalRead(74);
//        data = data | bit0;
//           data << 1;
//        
//
//  for( int mm=0; mm<3; mm++) {
//    endianSwap(data[mm].b);
//  }
//
//  return data;
//     
//}

float getIMU(byte commandToWrite, int x){

    //SPI.beginTransaction(settings);
//   float l_start = micros();
  /*Setup bytes to write*/
  // Clear the internal data buffer on the IMU
  byte result = transferByte(0x01);
      //Serial.print("Cleared internal buffer. Result: "),Serial.println(result);
     delay(1);

  // Send start of packet:
  result = transferByte(0xF6);
     //  Serial.print("Send start of packet. Result: "),Serial.println(result);
     delay(1);
  
  // Send command (tared euler angles)
  result = transferByte(commandToWrite); 
  //result=transferByte(0x01); //0x01 asks for tared Euler angles
     //  Serial.print("Send commmand 0x01. Result: "),Serial.println(result);
  
  // Get status of device:
  result = transferByte(0xFF);
     //  Serial.print("Status of device. Result: "),Serial.println(result);

  // idle represents whether or not the IMU is in the idle state. if it is then it will
  // breake the loop after counter reaches a number of cycles in the idle state   
  byte idle = 1;
  int counter = 0;
  while (result != 0x01 && (idle == 1 || counter < 11)) {  // Repeat until device is Ready 
    delay(1);
    result = transferByte(0xFF);
    //SerialUSB.print("Status of device. Result: "),SerialUSB.println(result);
    if (result == 0){
      idle = 0;
      counter ++;
    }
  }


  if (idle == 1){
    // Get the 12 bytes of return data from the device: 
    for (int ii=0; ii<3; ii++) {
      for (int jj=0; jj<4; jj++) {
        data[ii].b[jj] =  transferByte(0xFF);
      }
    }    
    //Swap bytes from big endian to little endian
    for(int mm=0; mm<3; mm++) {
      endianSwap(data[mm].b);
    }
    return data[x].fval;  //returns roll angle or roll rate
  }
  else{
    getIMU(commandToWrite, x);
  }
}

/* Retrieve data from IMU about roll angle and rate and return it */
struct roll_t updateIMUData() {
  roll_t roll_data;

  //get data from IMU
  float roll_angle = getIMU(0x01, 2);   //get roll angle
  float roll_rate = getIMU(0x26, 2);    //get roll rate
  float yaw = getIMU(0x01, 1); //get yaw
  roll_data.roll_angle = roll_angle;
  roll_data.roll_rate = roll_rate;
  roll_data.yaw = yaw;
  return roll_data;
}

/* Read from buffer. Used up UpdateIMUSerial */
void readBuffer(float dataArray[]) {
  int i = 0;
  String data;
  while(Serial1.available()) {
    if(Serial1.peek() == '\n') { //at end of response packet
      Serial1.read();
      if (i == 2) {
        dataArray[i] = data.toFloat(); //if last value in data, since no comma at end
      }
    }
    else {
      char ch = Serial1.read();
      if (ch == ',') { //delimiter between values
        dataArray[i] = data.toFloat();
        data = "";
        i++;
      }
      else data += ch;
    }
  }
}
/* Jordan and Bobby's failed semester-long attempt at switching to serial*/
void updateIMUDataSerial() {
  Serial1.write(":1\n"); //send command to get euler angles for orientation
  //Serial.println("Sent command for euler angles");
  readBuffer(euler_angles); //parse the 3 euler angles and put them into an array

  /*
  From our tests 5 ms is the minimum delay we need to give the IMU a chance to
  respond. At 4 ms or less we will get errors like "Lost sync with device"
  and "Serial Port read returned short ..."
  */
  delay(5);
  
  //Serial.println("Sent command for gyro");
  Serial1.write(":38\n"); //send command to get gyro rate
  readBuffer(gyro_rate); //parse the 3 gyro rates and put them into an array
  
  imu_data.roll_angle = euler_angles[2];
  imu_data.yaw = euler_angles[1];
  imu_data.roll_rate = gyro_rate[2];
  //Serial.print("Roll angle: ");
  //Serial.println(imu_data.roll_angle, 10);
  //Serial.print("Roll rate: ");
  //Serial.println(gyro_rate[2], 10);
  //Serial.print("Yaw: ");
  //Serial.println(imu_data.yaw, 10);
}


