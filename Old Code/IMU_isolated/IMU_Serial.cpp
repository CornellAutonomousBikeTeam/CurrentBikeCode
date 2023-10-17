#include "IMU_Serial.h"

/*IMU Setup and Functions*/
/////////////////////////////////////////////////////////////////////////////////////
union u_data {
    byte b[4];
    float fval;
} data[3]; // Create 3 unions, one for each value from the IMU
           // Euler angle: x-axis (pitch), y-axis (yaw), z-axis (roll)
           // Gyroscope: x-axis (pitch rate), y-axis (yaw rate), z-axis (roll rate)

//function to transfer commands through SPI
byte transferByte(byte byteToWrite) { 
  digitalWrite(1, LOW);    
  byte Result = 0x00;
  //Result = Serial1.write(byteToWrite);
  Result = SPI.transfer(byteToWrite); //Using SPI Transfer function to send instructions
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
  //Serial1.begin(115200);
  pinMode(1, OUTPUT); //cs
  pinMode(9, OUTPUT); //sck
  pinMode(8, OUTPUT); //mosi
  pinMode(10, INPUT); //miso

  digitalWrite(4, HIGH);
  SPI.begin();
  Serial.begin(9600);
}
float getIMU(byte commandToWrite, int x){
  byte result = transferByte(0x01);
  //System.out.println(result);
  delay(1);

  // Send start of packet:
  result = transferByte(0xF6);
 // System.out.println(result)
  delay(1);
  
  // Send command (tared euler angles)
  result = transferByte(commandToWrite); 
  //System.out.println(result);
  
  // Get status of device:
  result = transferByte(0xFF);
 // System.out.println(result);

  // idle represents whether or not the IMU is in the idle state. if it is then it will
  // breake the loop after counter reaches a number of cycles in the idle state   
  byte idle = 1;
  int counter = 0;
  while (result != 0x01 && (idle == 1 || counter < 11)) {  // Repeat until device is Ready 
    delay(1);
    result = transferByte(0xFF);
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
    for( int mm=0; mm<3; mm++) {
      endianSwap(data[mm].b);
    }
    
    return data[x].fval;  //returns roll angle or roll rate

  }
  else{
    getIMU(commandToWrite, x);
  }
}





