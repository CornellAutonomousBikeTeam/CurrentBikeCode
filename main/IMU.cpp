#include "IMU.h"

/*IMU Setup and Functions*/
/////////////////////////////////////////////////////////////////////////////////////
SPISettings settings(6000000, MSBFIRST, SPI_MODE0 ); //variable to hold SPI settings

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
////  SPI.transfer(10000); //streaming interval µS
////  SPI.transfer(0xFFFFFFFF, 4); //streaming duration
////  SPI.transfer(500);//Delay µS
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

float getIMU(byte commandToWrite){
    SPI.beginTransaction(settings);
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
//    Serial.print("Status of device. Result: "),Serial.println(result);
    if (result == 0){
      idle = 0;
      counter ++;
    }
  }
//  float l_diff = micros()- l_start;
//  Serial.println(l_diff);
  if (idle == 1){
    // Get the 12 bytes of return data from the device: 
    for (int ii=0; ii<3; ii++) {
      for (int jj=0; jj<4; jj++) {
        data[ii].b[jj] =  transferByte(0xFF);
      }
    }    
   
    SPI.endTransaction();
  
    //Swap bytes from big endian to little endian
    for( int mm=0; mm<3; mm++) {
      endianSwap(data[mm].b);
    }
    
    return data[2].fval;  //returns roll angle or roll rate

  }else{
    getIMU(commandToWrite);
  }
}




