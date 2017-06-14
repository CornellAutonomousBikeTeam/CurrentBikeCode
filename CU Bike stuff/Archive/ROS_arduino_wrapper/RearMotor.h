#ifndef RearMotor_h
#define RearMotor_h

#include <SPI.h>

/*Define definite variables*/
#define PWM_rear 8
#define hall_pin 11
#define reverse_pin 50

/*Define functions*/
//set speed
void rampToPWM(float, float);
//change direction
void switchDirection(boolean);


#endif //RearMotor_h
