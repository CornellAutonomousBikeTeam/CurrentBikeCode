#ifndef RearMotor_h
#define RearMotor_h

#include <SPI.h>
#include <Arduino.h>

#define PWM_rear 8
#define hall_pin 11
#define reverse_pin 50

// Rear wheel circumference, in meters
#define R_WHL_CIRCUMFERENCE 1.2446

//Rear Motor Variables
extern float rear_pwm; //current pwm value
extern double speed; //speed in rev/s
extern boolean forward; //if False, is running in reverse
extern int rwTickCount; // counts up for every interrupt

//Variables for calculating rear motor speed
extern float tNew; //last timestamp of interrupt

//Rear motor controller variable
extern float gain_p;
extern float desired_speed; //(m/s)

/*Define functions*/
//set speed
void rampToPWM(float, float);
//change direction
void switchDirection(boolean);
//handle interrupts coming from Hall sensor on rear wheel
// compute speed of rear wheel
void handleRWInterrupt();

#endif //RearMotor_h

