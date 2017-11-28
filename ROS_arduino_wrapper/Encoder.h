#ifndef Encoder_h
#define Encoder_h
#include <Arduino.h>

/*Variables*/
//Read the relative position of the encoder
extern signed int relativePos;
//Read the index value (Z channel) of the encoder
extern signed int indexValue;
extern const int quad_A;
extern const int quad_B;
extern const int idx;
extern const unsigned int mask_quad_A;
extern const unsigned int mask_quad_B;
extern const unsigned int mask_idx;
extern int REnot;
extern int DE;
extern signed int oldPosition;
extern signed int oldIndex;
extern unsigned long previous_t;
extern signed int x_offset;
extern float desired_pos;
extern float current_pos;
extern float current_vel;
extern float desired_vel;
extern float vel_error;
extern float pos_error;
extern float PID_output;
extern float sp_error;
extern float sv_error;
extern int pwm;


/*Functions*/
float updateEncoderPosition();

#endif //Encoder_h

