#ifndef FrontWheel_h
#define FrontWheel_h

#include <SPI.h>
#include <math.h>
#include "Encoder.h"

/*Define  variables*/
//PID
#define K_p 3000
#define K_d -40
#define K_i 0

extern const long interval; //Timed loop interval
extern float pid_controller_data_array[5];

//Front Motor
#define PWM_front 9
#define DIR 46

extern int steer_dir; 

extern float desired_steer; //Steer we want to achieve (either from RC or nav instruction)
extern float desired_lean; //Lean we want to achieve (either from RC or nav instruction)

extern int maxfront_PWM; //Define max front wheel PWM

//Balance Control constants
extern const int k1; //phi = lean
extern const int k2; //was previously 21 //phidot=lean rate
extern const int k3; //delta=steer

/*Define functions*/

/*
 * Runs a PID controller to keep the front wheel at desired_pos.
 *
 * Some notes on the pid_controller_data array: It's allocated (or at
 * least defined) by the caller and used to return debug info. It
 * contains a number of intermediate results created while computing the
 * PWM output to send the front motor. Its individual elements are
 * assigned as follows:
 *
 * {
 *     current_pos,
 *     desired_pos,
 *     current_vel,
 *     sp_error,
 *     sv_error,
 *     total_error
 * }
 */
float PID_Controller(float, signed int, signed int, 
  unsigned long, unsigned long, signed int, float *);

  
/*
 * Takes in commanded velocity from balance controller, and converts
 * commanded velocity into commanded position
 */
float eulerIntegrate(float, float);

/* Takes in desired position and applies a PID controller to minimize
 * error between current position and desired position. This function
 * also calls PID_Controller function, which sends the actual PWM
 * signal to the front wheel.
 */
float frontWheelControl(float, float);

/* Function that returns desired angular velocity of front wheel based on roll angle, 
 *  roll rate (both from IMU) and encoder angle as well as desired lean and steer */
float balanceController(float, float, float );

#endif //PID_h

//Gameplan:
//Try out different gains off ground for holding position (stationary testing)
//Try out different gains ON ground (max 45 minutes) (step input)
//if failed, try different gains OFF ground, tune (step input)
