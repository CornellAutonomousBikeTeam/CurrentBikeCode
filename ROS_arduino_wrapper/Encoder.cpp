#include "Encoder.h"

/*Variables*/
signed int relativePos = REG_TC0_CV0;
signed int indexValue = REG_TC0_CV1;
const int quad_A = 2;
const int quad_B = 13;
const int idx = 60;
const unsigned int mask_quad_A = digitalPinToBitMask(quad_A);
const unsigned int mask_quad_B = digitalPinToBitMask(quad_B);
const unsigned int mask_idx = digitalPinToBitMask(idx);
int REnot = 3;
int DE = 4;
signed int oldPosition  = 0;
signed int oldIndex = 0;
unsigned long previous_t = 0;
signed int x_offset = 0; 
float desired_pos = 0;
float current_pos = 0;
float current_vel = 0;
float desired_vel = 0;
float vel_error = 0;
float pos_error = 0;
float PID_output = 0;
float sp_error = 0;
float sv_error = 0;
int pwm = 0;

/*Functions*/
float updateEncoderPosition() {
  relativePos = REG_TC0_CV0;
  indexValue = REG_TC0_CV1;
  current_pos = (((relativePos - x_offset) * 0.02197 * M_PI) / 180); //Angle (rad)
  return current_pos;
}

