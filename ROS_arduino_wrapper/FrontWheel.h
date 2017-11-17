#ifndef FrontWheel_h
#define FrontWheel_h

#include <Encoder.h> //In order to use relativePos and indexValue
#include <SPI.h>
#include <math.h>

/*Define definite variables*/

#define interval 10000

//PID
#define PWM_front 9
#define DIR 46
#define K_p 3000
#define K_d -40
#define K_i 0

/*Define functions*/


float eulerIntegrate(float desiredVelocity, float current_pos);


void frontWheelControl(float desiredVelocity, float current_pos);


//PID
float PID_Controller(float, signed int, signed int, 
  unsigned long, unsigned long, signed int, float *);


#endif //PID_h

