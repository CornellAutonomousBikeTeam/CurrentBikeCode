#include "FrontWheel.h"
#include <math.h>

/*Variables*/
const long interval = 10000;
std_msgs::Float32MultiArray pid_controller_data;
int steer_dir = 0;

int duration_CH1;
int duration_CH2;
int duration_CH3;
int duration_CH4;
int duration_CH5;
int duration_CH6;

int start_CH1;
int start_CH2; 
int start_CH3;
int start_CH4;
int start_CH5;
int start_CH6;

int end_CH1;
int end_CH2; 
int end_CH3;
int end_CH4;
int end_CH5;
int end_CH6;

float desired_steer = 0;
float desired_lean = 0;
float desired_pos_array[250];
float theo_position = 0;

int maxfront_PWM = 110;

const int k1 = 70;
const int k2 = 10; 
const int k3 = -20; 

/*Functions*/
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
  float current_vel = (((((x-x_offset)-oldPosition)*0.02197*1000000*M_PI/180.0)/(current_t-previous_t)));   //Angular Speed(rad/s)
  pid_controller_data[2] = current_vel;
  
  //calculate the value of the current time step in microseconds
  //unsigned long delta_t = 2000;
  
  // the value of the velocity error will be negative of the current velocity (in order to resist current direction of motion). 
  //Calculated as target_velocity - current_velocity where target velocity is always 0
  //scaled velocity error
  float sv_error =  (-K_d*current_vel)  ;  
  pid_controller_data[4] = sv_error;

  float total_error =  sp_error + sv_error; //Total error: scaled velocity and positional errors
  pid_controller_data[5] = total_error;

  if (total_error > 0) {digitalWrite(DIR, LOW);} //Direction of front wheel's rotation
  else {digitalWrite(DIR, HIGH);}

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

float eulerIntegrate(float desiredVelocity, float current_pos) {
  float desiredPosition = current_pos + desiredVelocity * ((float)interval / 1000000.0) ;
  return desiredPosition;
}

float frontWheelControl(float desiredVelocity, float current_pos) {

  unsigned long current_t = micros();
  float desired_pos = eulerIntegrate(desiredVelocity, current_pos);

  // The PID_Controller function will actually rotate the front motor!
  float pid_controller_data_array[5];
  float current_vel = PID_Controller(desired_pos, relativePos, x_offset, current_t, previous_t, oldPosition, pid_controller_data_array);

  // Copy data from the PID controller into the outgoing ROS topic structure
  for(int i = 0; i < 6; i++) {
    pid_controller_data.data[i] = pid_controller_data_array[i];
  }

  previous_t = current_t;
  oldPosition = relativePos - x_offset;
}

float balanceController(float roll_angle, float roll_rate, float encoder_angle) {
  float desiredSteerRate = k1 * (roll_angle - desired_lean) + k2 * roll_rate + 
    k3 * (encoder_angle - desired_steer);
  if (desiredSteerRate > 10) {
    desiredSteerRate = 10;
  }
  else if (desiredSteerRate < -10) {
    desiredSteerRate = -10;
  }
  return desiredSteerRate;
}

