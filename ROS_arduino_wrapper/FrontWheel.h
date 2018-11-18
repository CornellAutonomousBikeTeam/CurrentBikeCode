#ifndef FrontWheel_h
#define FrontWheel_h

#include <SPI.h>
#include <math.h>
#include <std_msgs/Float32MultiArray.h>
#include "Encoder.h"

/*Define  variables*/
//PID
#define PWM_front 9
#define DIR 46
#define K_p 3000
#define K_d -60
#define K_i 0

extern const long interval;
extern std_msgs::Float32MultiArray pid_controller_data; //Array containing pid controller debug variables

//Front Motor
#define PWM_front 9
#define DIR 46

extern int steer_dir;

//timers for each channel
extern int duration_CH1;
extern int duration_CH2;
extern int duration_CH3;
extern int duration_CH4;
extern int duration_CH5;
extern int duration_CH6;

extern int start_CH1;
extern int start_CH2; 
extern int start_CH3;
extern int start_CH4;
extern int start_CH5;
extern int start_CH6;

extern int end_CH1;
extern int end_CH2; 
extern int end_CH3;
extern int end_CH4;
extern int end_CH5;
extern int end_CH6;

extern float desired_steer;
extern float desired_lean;
extern float desired_pos_array[250];
extern float theo_position;
extern float des_pos;

extern int maxfront_PWM; //Define max front wheel PWM

//Balance Control constants
extern const int k1; //phi = lean
extern const int k2; //was previously 21 //phidot=lean rate
extern const int k3; //delta=steer

/*Define functions*/
float PID_Controller(float, signed int, signed int, 
  unsigned long, unsigned long, signed int, float *);
/*
 * Takes in commanded velocity from balance controller, and converts
 * commanded velocity into commanded position
 */
float eulerIntegrate(float, float);
/*
 * Takes in desired position and applies a PID controller to minimize
 * error between current position and desired position. This function
 * also calls PID_Controller (from PID.cpp), which sends the actual PWM
 * signal to the front wheel.
 */
float frontWheelControl(float, float, float);

/* Function that returns desired angular velocity of front wheel */
float balanceController(float, float, float );

#endif //PID_h

//Gameplan:
//Try out different gains off ground for holding position (stationary testing)
//Try out different gains ON ground (max 45 minutes) (step input)
//if failed, try different gains OFF ground, tune (step input)
