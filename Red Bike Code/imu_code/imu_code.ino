#include <SPI.h>
/*
Alex Li, al2356
I worked on this IMU code and commented in detail some of the parts that needed more explaination or were more intricate. This includes what some of the functions do, what processes are going, and what some words mean.
I tried to write everything in a way that anyone reading the code could kind of figure out what is going and the purpose of our code. Since this code was made for IMU there are a lot of specific terms and functions 
that were used. I also explained why we used certain functions instead of others to make things more clear. This commenting will also contribute in a way to my metacognitive goal of self-monitoring and understanding the
code for this semester.
*/
SPISettings settings(6000000, MSBFIRST, SPI_MODE0);  //variable to hold SPI settings
//Set Slave Select, MOSI, MISO, CLK
const int CSN = 1;       //chip select
const int SO = 10;       //MISO
const int SI = 8;        //MOSI
const int CLK = 9;       //Serial Clock
const int delta = division(180, 3.14)   
const byte INST = 0x01;  //Read filtered, tared orientation (Euler Angles)// 0x01, 0x02, 0x04, 0x08 (3, 6, 9, 10)
const byte instructions[3] = {0x01, 0x02, 0x04, 0x08}
const float flinst[3] = {3, 9, 6, 9};
const int instruction_number = 0;
unsigned long startMillis; //preset to 0
unsigned long currentMillis; //preset to 0
// Needed to convert the bytes from SPI to float
union u_types {
  byte b[4];
  float fval;
} data[3];  // Create 3 unions, one for each euler angle

void setup() {
  //Set Pin Modes
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN, HIGH);
  //Initialize SPI
  SPI.begin();
  //pour a bowl of serial
  Serial.begin(9600);
}

//function to transfer commands through SPI
/*
This function takes a single byte of data performs some sort of operation on it then returns the another byte of data as the output. 
Input: Any instructions. In our case we are using Euler angles.
Output: Sucess or Fail
*/
byte transferByte(byte byteToWrite) {

  byte Result = 0x00;
  digitalWrite(CSN, LOW);
  delay(1);
  Result = SPI.transfer(byteToWrite); 
  delay(1);
  digitalWrite(CSN, HIGH);
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
//Prep. Calibrate. Send. Check. Function to ensure the data is sent correctly.
byte readData(byte instruction) {
  //comment this transfer byte
  byte result = transferByte(0x01);
  Serial.print("Cleared internal buffer. Result: "), Serial.println(result);

  // Send start of packet:
  result = transferByte(0xF6);
  Serial.print("Send start of packet. Result: "), Serial.println(result);

  // Send command (tared euler angles)
  result = transferByte(0x07);
  Serial.print("Send commmand 0x01. Result: "), Serial.println(result); // what this line does

  // Get status of device:
  result = transferByte(0xFF);
  Serial.print("Status of device. Result: "), Serial.println(result);

  //delay(1);
  /*
  while (result != 0x01) {  // Repeat until device is Ready
    delay(1);
    result = transferByte(0xFF);
    Serial.print("Status of device. Result: "),Serial.println(result);
  }*/

  return result;
}
void loop() {
  //Begin new SPI Transaction
  SPI.beginTransaction(settings);

  byte result_loop = readData(instructions[instruction_number]);
  //We use a while loop because we don't know how many iterations will be required before the device is ready. 
  while (result_loop != 0x01) {  // Repeat until device is Ready
    delay(1);
    result_loop = transferByte(0xFF);
    //Serial.print("Status of device. Result: "), Serial.println(result_loop);
  }

  //Serial.println("Count out");
  for (int i = 0; i < (flinst[instruction_number]); i++) {
    for (int j = 0; j < 4; j++) {
      data[i].b[j] = transferByte(0xFF);  // Transfer a byte of data over the SPI interface and store it in the `data` array
      delay(1);                           //delay to ensure byte is recived before sending the next one
    }
  }

  //If result is =0x01, execute the following
  /*
  if (result_loop == 0x01){
    //Reads 12 bytes of data, 4 bytes at a time, and stores in 'data' array of structure
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 4; j++){
        data[i].b[j] =  transferByte(0xFF); // Transfer a byte of data over the SPI interface and store it in the `data` array
        delay(1); //delay to ensure byte is recived before sending the next one
      }
    }
  }

*/
  SPI.endTransaction();  // End current SPI transaction
  if (result_loop == 0x01) {
    for (int mm = 0; mm < 3; mm++) {
      endianSwap(data[mm].b);  //Call the `endianSwap` function to swap the byte order of each element in the `data` array
    }

    //// Output the values of three floating point variables to the serial port
    //Initialize with start time and current time in milliseconds. Then subtracts the two to find the time that has elapsed. 
    startMillis = micros();
    Serial.println(startMillis);
    data[0].fval = data[0].fval * (180/3.14); //current_radians value * (180/3.14) to get degrees 
    data[1].fval = data[1].fval * (180/3.14); //current_radians value * (180/3.14) to get degrees 
    data[2].fval = data[2].fval * (180/3.14); //current_radians value * (180/3.14) to get degrees 
    currentMillis = micros();
    Serial.println(currentMillis);
    Serial.print("Time Diff All:"), Serial.println(currentMillis - startMillis);

    //Serial.print("fval 1:"), Serial.println(data[0].fval); //z - yaw
    //Serial.print("fval 2:"), Serial.println(data[1].fval); //y degrees - pitch
    //Serial.print("fval 3:"), Serial.println(data[2].fval);//x - roll


  }
  delay(3000);
}