#ifndef RearMotor_h
#define RearMotor_h

#include <SPI.h>
#include <Arduino.h>

/*Define definite variables*/
#define PWM_rear 8
#define hall_pin 11
#define reverse_pin 50

//Rear Motor Variables
extern float rear_pwm; //current pwm value
extern double speed; //speed in rev/s
extern boolean forward; //if False, is running in reverse

//Variables for calculating rear motor speed
extern float tOld; //first time reading
extern float tNew; //second time reading
extern double T; //holds difference between tOld and tNew

//Rear motor controller variables
extern float gain_p; //gain
extern float desired_speed; //(m/s)

/*
   Method for setting rear motor at a certain PWM
   @param newPWM- the new pwm value to ramp up to
*/
void rampToPWM(float, float);
/* Switches direction of motor */
void switchDirection(boolean);
/* Calculates period of wheel turning */
void getPeriod();

#endif //RearMotor_h

