#define Center_Point         1500

unsigned int ReqSpeed = 710;

#include <SAMD21turboPWM.h>



#define REARPWM       3

//creates pwm instance
TurboPWM pwm;

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
#define D11 mkpin(A, 08)
#define D12 mkpin(A, 09)
#define D13 mkpin(B, 23)
#define D14 mkpin(B, 22)
#define D15 mkpin(A, 02)
#define A0 D15
#define D16 mkpin(B, 02)
#define A1 D16
#define D17 mkpin(B, 03)
#define A2 D17
#define D18 mkpin(A, 04)
#define A3 D18
#define D19 mkpin(A, 05)
#define A4 D19
#define D20 mkpin(A, 06)
#define A5 D20
#define D21 mkpin(A, 07)
#define A6 D21

#define OutSet(pin) (PORT_IOBUS->Group[pin>>5].OUTSET.reg = 1<<(pin & 0x1f))  // Make high
#define OutClr(pin) (PORT_IOBUS->Group[pin>>5].OUTCLR.reg = 1<<(pin & 0x1f))  // Make low
#define OutTgl(pin) (PORT_IOBUS->Group[pin>>5].OUTTGL.reg = 1<<(pin & 0x1f))  // toggle value

#define Output(pin)  (PORT_IOBUS->Group[pin>>5].DIRSET.reg = 1<<(pin & 0x1f))  // Make output
#define Input(pin) (PORT_IOBUS->Group[pin>>5].DIRCLR.reg = 1<<(pin & 0x1f))  // Make input
// #define DIRTgl(pin) (PORT_IOBUS->Group[pin>>5].DIRTGL.reg = 1<<(pin & 0x1f))  // toggle in/out

#define Read(pin)  ((PORT_IOBUS->Group[pin>>5].IN.reg) >> (pin & 0x1f) & 1)   // Read pin


uint8_t channel = 0;

void setup()
{
  Serial.begin(1000000);
  Output(D5);
  //assigns PWM frequency of 1.0 KHz and a duty cycle of 0%
  pwm.setClockDivider(1, true);
  pwm.timer(1, 1, 100, true);
}

void loop()
{
  int shifted = ReqSpeed - Center_Point;
  if(shifted<0){
    shifted = 0-shifted;
    OutSet(D5);
  }else{
    OutClr(D5);
  }
  pwm.analogWrite(REARPWM, shifted);

}
