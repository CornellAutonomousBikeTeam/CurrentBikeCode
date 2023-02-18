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
/*
byte transferInst(byte inst, int cs){
  digitalWrite(4, LOW);
  //OutClr(D4);
  delay(1);
  byte res = SPI.transfer(inst);
  delay(1);
  //OutSet(D4);
  digitalWrite(4, HIGH);
  return res;
}*/
byte transferInst(byte inst){
  return Serial1.write(inst);
}
void setup() {
  /*
  Output(D4); //CS - could hardwire later - could also be the current problem
  Output(D9); //SCK
  Output(D8); //MOSI
  Input(D10);//MISO
  OutSet(D4);//active low so pull high to disable initially]
  */
  /*
  pinMode(4, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, INPUT);*/
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  SPI.begin();// start SPI
  Serial.begin(9600);
} 
/*
 * SPI transfer function for any select line. Takes in select line)
 */
void readData(){
  for (int i=0; i<3; i++) {
    for (int j=3; j>=0; j--) {
      imu_d[i].b_imu[j] =  transferInst(0xFF, cs);
    }
  }  
}
/*
void loop() {
  // put your main code here, to run repeatedly:
  SPI.beginTransaction(SPISettings(DATA_RATE, MSBFIRST, SPI_MODE0));

  byte buffer_clear_result = transferInst(0x01, cs);
  Serial.print("Buffer Clear Result"),Serial.println(buffer_clear_result, HEX);
  byte packet_result = transferInst(0xF6, cs);
  Serial.print("Packet Result"),Serial.println(packet_result, HEX);
  byte command_result = transferInst(0x01, cs);
  Serial.print("Command Result"),Serial.println(command_result, HEX);
  byte status_val = transferInst(0XFF, cs);
  Serial.println(status_val, HEX);
  if(status_val == 0x01){
    readData();
  }
  for(int k = 0; k < 3; k++){
    Serial.print("Degree Tared: ");
    Serial.println(imu_d[k].f_imu);
  }
  
}
*/
void loop(){
  byte buffer_clear_result = transferInst(0x01);
  delay(1);

  byte packet_result = transferInst(0xF6);
  delay(1);
  
  byte command_result = transferInst(0x01);
  delay(1);

  byte status_val = transferInst(0xFF);

  if (status_val == 0x01){
    readData();
  }
  
}
