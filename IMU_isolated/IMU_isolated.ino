#include "IMU_Serial.h"
const byte numChars = 26;
char receivedChars[numChars]; // an array to store the received data

boolean newData = false; //Whether Arduino reached the end of the message


void setup() {
  // put your setup code here, to run once:
  initIMU();
  Serial.begin(9600); //set baud rate to 9600
    // A struct to hold the tared euler angles from IMU
  struct tared {
    float rate;
    float angle;
    float yaw; 
  };
  // A struct to hold the gyroscope info
  struct gyro {
    float axis1;
    float axis2;
    float axis3;
  };
  
  /*
   * 80 (0x50) means "Set streaming slots"
   * 44 means it will run the command "Get temperature F"
   * 255 is used to fill the remaining command slots (8 slots total)
   */
  Serial1.write(":80,1,38,255,255,255,255,255,255\n");
  Serial1.write(":221,4\n");
  /*
   * Command byte: 82 (0x52) means "Set streaming timing"
   * 1st parameter: 20000 means it will output data 50 times 
   *  a second (0 means as fast as possible)
   * 2nd parameter: how long session will run; -1 means the 
   *  session will run indefinitely until a stop streaming command is 
   *  explicitly issued
   * 3rd parameter: delay in ms before beginning streaming; 0 means 
   *  the start delay is 0 milliseconds before the sensor actually 
   *  begins streaming
   */
  Serial1.write(":82,20000,-1,0\n");
  /*
   * First byte: semicolon means return with response header
   *  specified in command 221 above
   * Second byte: 85 means start streaming session
   */
  Serial1.write(";85\n");//Start the streaming session with response header
  //Serial1.write(":85\n");//Start the streaming session without response header
  
}
void loop() {
 recvWithStartEndMarkers();
 showNewData();
}

void recvWithStartEndMarkers() {
 //static keyword is used to create variables for one function
 //They are different from local variables in that they persist between calls
 static boolean recvInProgress = false;
 static byte index = 0;
 char startMarker = '\0';
 char endMarker = '\n';
 char rc; //Received character
 
 // if (Serial.available() > 0) {
 /*
 Serial.available() is the data already stored in the serial receive buffer (64 bytes)
 Serial.available() > 0 means that there is data available for reading
 newData is True when Arduino received the full message (hit the endMarker \n)
 */
 while (Serial1.available() > 0 && newData == false) {
  rc = Serial1.read(); //Read the next character in the serial buffer

  //If reading the message
  if (recvInProgress == true) {
    //The next character is not \n
    if (rc != endMarker) {
      receivedChars[index] = rc; //Add the next character to the array
      index++; //Move the index to the next character
      /*
      If we haven't hit the end of the message but have read 12 characters, keep
      reading for the last character
      */
      if (index >= numChars) {
        index = numChars - 1;
      }
     }
     //If we reached the \n character
     else {
      receivedChars[index] = '\0'; // terminate the string with null character
      recvInProgress = false; //Done reading this message
      index = 0; //Reset the index
      newData = true; //Arduino has received the full message
     }
   }
   //If the next character indicates the start of a message, read the message
   else if (rc == startMarker) {
      recvInProgress = true; 
   }
  }
}

void showNewData() {
 if (newData == true) {
 Serial.print("This just in ... ");
 Serial.println(receivedChars);
 newData = false;
 }
}
