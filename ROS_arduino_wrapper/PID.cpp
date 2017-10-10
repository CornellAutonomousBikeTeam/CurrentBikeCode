#include "PID.h"
#include <math.h>

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
float PID_Controller(float desired_pos, signed int x, signed int x_offset, 
  unsigned long current_t, unsigned long previous_t, signed int oldPosition,
  float *pid_controller_data) {

  //when y value changes (when wheel passes index tick) print absolute position of the wheel now to see if encoder absolute position
  //is drifting
  float current_pos = (((x - x_offset) * 0.02197 * M_PI)/180); //Angle (rad)
  pid_controller_data[0] = current_pos;

  //write PID controller based off of error signal received from encoder
  //P term
  //calculate position error (rad)
  float pos_error = desired_pos - current_pos ;
  pid_controller_data[1] = desired_pos;

  //scaled positional error
  //position scaling factor K_p = 100/(M_PI/2) found by taking 100 (100 being max pwm value I want to reach), 
  //and dividing by theoretical max absolute value of angle (Pi/2). 
  //This means with angles in that range, 100 will be the max PWM value outputted to the motor
  float sp_error =  (K_p*pos_error);
  pid_controller_data[3] = sp_error;

  //D term
  //calculate velocity error
//  unsigned long current_t = micros();
  float current_vel = (((((x-x_offset)-oldPosition)*0.02197*1000000*M_PI/180.0)/(current_t-previous_t)));   //Angular Speed(rad/s)
  pid_controller_data[2] = current_vel;
  
  //calculate the value of the current time step in microseconds
  //unsigned long delta_t = 2000;
  
  // the value of the velocity error will be negative of the current velocity (in order to resist current direction of motion). 
  //Calculated as target_velocity - current_velocity where target velocity is always 0
  //scaled velocity error
  float sv_error =  (-K_d*current_vel)  ;  
  pid_controller_data[4] = sv_error;

  float total_error =  sp_error + sv_error ;
  pid_controller_data[5] = total_error;

  //print total error to get a sense of how high the values are for a normal sine wave.

  // This pin sets the direction of the front wheel's rotation
  if (total_error > 0) {
    digitalWrite(DIR, LOW); 
  } else {
    digitalWrite(DIR, HIGH);
  }

 //  Serial.println(String(current_pos) + "\t" + String(desired_pos) + "\t" + String(pos_error) + "\t" + String(total_error));

  oldPosition = x-x_offset;

  // Cast the output to the motor to an int
  int motor_output = (int)total_error;

  // We only want the magnitude of the output
  motor_output = abs(motor_output);

  // Maximium motor output magnitude should be 100
  if(motor_output > 100) {
    motor_output = 100;
  //This maybe should be increased to 255 with the repaired front motor 10.10.17
  }

  // Write to the motor
  analogWrite(PWM_front, motor_output);
  return current_vel;
}

