#ifndef Encoder_h
#define Encoder_h
#include <Arduino.h>

/*Variables*/
extern signed int relativePos; //Read the relative position of the encoder
extern signed int indexValue; //Read the index value (Z channel) of the encoder
extern const int quad_A; //Used to assign mask_quad_A
extern const int quad_B; //Used to assign mask_quad_B
extern const int idx; //Used to assign mask_idx
extern const unsigned int mask_quad_A; //Used to activate peripheral function for the quad pin it's assigned to
extern const unsigned int mask_quad_B;//Used to activate peripheral function for the quad pin it's assigned to
extern const unsigned int mask_idx; //Used to activate peripheral function for the quad pin it's assigned to
extern int REnot; //Pin used by encoder 
extern int DE; //Pin used by encoder 
extern signed int oldPosition; //Used by front wheel
extern signed int oldIndex; //Used in the front wheel calibration loop in the main file
extern unsigned long previous_t; //Used by front wheel
extern signed int x_offset; //Where the front tick is with respect to the absolute position of the encoder A and B channels
extern float desired_pos; //Used by PID controller
extern float current_pos; //Used to update encoder position
extern float current_vel; //Holds desired velocity but modified to match sign convention
extern float desired_vel; //Does not seem to be used anywhere
extern float vel_error; //Does not seem to be used anywhere
extern float pos_error; //Used to calculate position error for PID
extern float PID_output; //Does not seem to be used anywhere
extern float sp_error; //Used in scaled positional error for PID
extern float sv_error; //Used in scaled velocity error for PID
extern int pwm; //PWM for front motor


/*Functions*/
float updateEncoderPosition(); //Updates global variables representing encoder position

#endif //Encoder_h

