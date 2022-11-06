#include <SPI.h>
//Fast register based mapping
#define A 0
#define B 1
#define mkpin(port, pin) ((port<<5) + pin)

#define D0  mkpin(A, 22)
#define D1  mkpin(A, 23)
#define D2  mkpin(A, 10)
#define D3  mkpin(A, 11)
#define D4  mkpin(B, 10)
#define D5  mkpin(B, 11)
#define D6  mkpin(A, 20)
#define D7  mkpin(A, 21)
#define D8  mkpin(A, 16)
#define D9  mkpin(A, 17)
#define D10 mkpin(A, 19)
#define D11 mkpin(A, 8)
#define D12 mkpin(A, 9)
#define D13 mkpin(B, 23)
#define D14 mkpin(B, 22)
#define D15 mkpin(A, 2)
#define A0 D15
#define D16 mkpin(B, 2)
#define A1 D16
#define D17 mkpin(B, 3)
#define A2 D17
#define D18 mkpin(A, 4)
#define A3 D18
#define D19 mkpin(A, 5)
#define A4 D19
#define D20 mkpin(A, 6)
#define A5 D20
#define D21 mkpin(A, 7)
#define A6 D21

#define OutSet(pin) (PORT_IOBUS->Group[pin>>5].OUTSET.reg = 1<<(pin & 0x1f))  // Make high
#define OutClr(pin) (PORT_IOBUS->Group[pin>>5].OUTCLR.reg = 1<<(pin & 0x1f))  // Make low
#define OutTgl(pin) (PORT_IOBUS->Group[pin>>5].OUTTGL.reg = 1<<(pin & 0x1f))  // toggle value

#define Input(pin)  (PORT_IOBUS->Group[pin>>5].DIRSET.reg = 1<<(pin & 0x1f))  // Make input
#define Output(pin) (PORT_IOBUS->Group[pin>>5].DIRCLR.reg = 1<<(pin & 0x1f))  // Make output
// #define DIRTgl(pin) (PORT_IOBUS->Group[pin>>5].DIRTGL.reg = 1<<(pin & 0x1f))  // toggle in/out

#define Read(pin)  ((PORT_IOBUS->Group[pin>>5].IN.reg) >> (pin & 0x1f) & 1)   // Read pin

//end of fast register based mapping

const byte CLEAR = 0x01;
const byte COMMAND = 0x01;
const byte RESULT_COMM = 0XFF;
const byte PACKET_START = 0xF7;
int cs = 20; //D11
int mosi = 0;//3 over from the end
int miso = 0;
int clk = 0;
int inter = 0; //TxD - interrupt for data ready - one of the 9 hardware interrupt pins
const int DATA_RATE = 6000000; // think its 6 MHz
bool readFlag;
//MOSI, SCK, MISO, CS (3 over from the end for MOSI) - D8, D9, D10, D11 - chip select 20, miso 19, 18 SCK, 17 MOSI


union imu_data{
  byte b_imu[4];
  float f_imu;
}imu_d[3];

byte transferInst(byte inst, int cs){
  //digitalWrite(cs, LOW);
  OutClr(D11);
  byte res = SPI.transfer(inst);
  OutSet(D11);
  return res;
}

void readStatus(){
  readFlag = true;
}
void setup() {
  // put y our setup code here, to run once:
  //SPI Setup
  //pinMode(cs, OUTPUT);
  Output(D11);
  //pinMode(inter, INPUT);
  //attachInterrupt(digitalPinToInterrupt(inter), readStatus, HIGH); 
  Outset(D11);
  SPI.begin();
  Serial.begin(9600);
  
} 
/*
 * SPI transfer function for any select line. Takes in select line)
 */
void readData(){
  for (int i=0; i<3; i++) {
    for (int j=0; j<4; j++) {
      imu_d[i].b_imu[j] =  transferInst(0xFF, cs);
    }
  }  
}
void loop() {
  // put your main code here, to run repeatedly:
  SPI.beginTransaction(SPISettings(DATA_RATE, MSBFIRST, SPI_MODE0));
  //add 
  /*
   * Send initial instructions for instruction transfer
   */
  transferInst(0x01, cs);
  delay(1);//sample millisecond delay to avoid potential corrupted transactions
  transferInst(0xF6, cs);
  delay(1);//need to add delays? //do in delay where the status isn't ready. 
  byte status_val = transferInst(0XFF, cs);
  delay(1);
  //Read data from YEI 3 Space sensor - need to do endian swap - not sure
  //read on flag high
  /*
  if (readFlag){
    readStatus();
    readFlag = false; //reset flag
  }
  */
  if(status_val != 0x01){
    readData();
  }
}
