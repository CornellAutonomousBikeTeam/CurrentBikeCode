#include "Front_Motor.h"

/*Define definite variables*/
//Front Motor
#define PWM_front 9
#define DIR 46

//These really don't need to be defined... I have created constructors instead so
//it's easier to change these constants
//#define K_p 2000
//#define K_d -10
//#define K_i 0

/*
   Constructor implementation
*/
Front_Motor_Controller::Front_Motor_Controller(int p, int d, int i): K_p(p), K_d(d), K_i(i) {};
/**
   Convenience method that sets pins and encoder values
*/
void Front_Motor_Controller::encoderAndFrontMotorSetup() {
  //setup Encoder
  pinMode(REnot, OUTPUT);
  pinMode(DE, OUTPUT);

  // activate peripheral functions for quad pins
  REG_PIOB_PDR = mask_quad_A;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_A;   // choose peripheral option B
  REG_PIOB_PDR = mask_quad_B;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_quad_B;   // choose peripheral option B
  REG_PIOB_PDR = mask_idx;     // activate peripheral function (disables all PIO functionality)
  REG_PIOB_ABSR |= mask_idx;   // choose peripheral option B


  // activate clock for TC0 and TC1
  REG_PMC_PCER0 = (1 << 27) | (1 << 28) | (1 << 29);

  // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = 5;


  // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1 << 9) | (1 << 8) | (1 << 12);


  // activate the interrupt enable register for index counts (stored in REG_TC0_CV1)
  REG_TC0_QIER = 1;


  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  // SWTRG = 1 necessary to start the clock!!
  REG_TC0_CCR0 = 5;
  REG_TC0_CCR1 = 5;

  //setup Motor Outputs
  pinMode(DIR, OUTPUT);
  pinMode (PWM_front, OUTPUT);
}




/*
   Convenience method that runs calibration code for front motor
*/
void Front_Motor_Controller::calibrate() {
  //  the follwing loop will not terminate until wheel passes front tick on encoder twice. The second time should be passed very slowly-
  //  this will allow for the most accurate location to be found for the center alignment of the front wheel with the bike.

  signed int y = REG_TC0_CV1;
  oldIndex = y;
  digitalWrite(DIR, HIGH);
  while (y == oldIndex) {
    analogWrite(PWM_front, 20);
    y = REG_TC0_CV1;
    //Serial.println("Ticking");
  }

  //set x offset to define where the front tick is with respect to the absolute position of the encoder A and B channels
  x_offset = REG_TC0_CV0;
  analogWrite(PWM_front, 0);
}

/*
   Helper methods that calculate various conversions
*/

/* takes in desired angular velocity returns pwm */
int Front_Motor_Controller::velocityToPWM (float desiredVelocity) {
  battery_voltage = analogRead(VOLTAGE_PIN);
  //Serial.println("pin 63 output " + String(battery_voltage));
  battery_voltage = battery_voltage / VOLTAGE_CONST;

  //Serial.println("voltage is " + String(battery_voltage));
  pwm = 256 * (desiredVelocity - VELOCITY_VOLTAGE_C) / (battery_voltage * VELOCITY_VOLTAGE_K);
  //Serial.println("pwm is  " + String(pwm));

  if (desiredVelocity > 18 ) { //***TO DO*** THIS LIMITATION MUST GO ON ALL OF THE PWM GOING TO THE FRONT MOTOR, NOT JUST THE FEED FORWARD LOOP
    //put in the warning
    return maxfront_PWM;
  } else {
    return pwm;
  }
}


/* intakes commanded velocity from balance controller
   converts commanded velocity into commanded position */
float Front_Motor_Controller::eulerIntegrate(float desiredVelocity, float current_pos) {
  float desiredPosition = current_pos + desiredVelocity * ((float)interval / 1000000.0) ;
  return desiredPosition;
}
/*
   End calculation helper methods
*/







/*
   updates global variables representing encoder position
*/
float Front_Motor_Controller::updateEncoderPosition() {
  //Read the relative position of the encoder
  relativePos = REG_TC0_CV0;
  //Read the index value (Z channel) of the encoder
  indexValue = REG_TC0_CV1;
  current_pos = (((relativePos - x_offset) * 0.02197 * M_PI) / 180); //Angle (rad)
  return current_pos;
}








/* takes in desired position and applies a PID controller to minimize error between current position and desired position */
float Front_Motor_Controller::frontWheelControl(float desiredVelocity, float current_pos) { //steer contribution doese not need to be passed into
  //frontWheelControl because it is a global variable

  unsigned long current_t = micros();

  //  if (n == 0) {
  //    float desired_pos = 0;
  //    PID_Controller(desired_pos, relativePos, x_offset, current_t, previous_t, oldPosition);
  //    n++;
  //  }
  float desired_pos = eulerIntegrate(desiredVelocity, current_pos);
  //Serial.println(String(theo_position) + '\t' + String(desired_pos) + '\t' + String(current_pos)) ;

  /*
    if (Serial.available()){
    desired_pos = M_PI / 180 * Serial.parseFloat();
    }
  */



  //Serial.println(String(steer_contribution) + '\t' +  String(commanded_speed));

  float current_vel = PID_Controller(desired_pos, relativePos, x_offset, current_t, previous_t, oldPosition);

  previous_t = current_t;
  oldPosition = relativePos - x_offset;
}







/* FUNCTION THAT RETURNS DESIRED ANGULAR VELOCITY OF FRONT WHEEL */
float Front_Motor_Controller::balanceController(float roll_angle, float roll_rate, float encoder_angle) {
  float desiredSteerRate = (k1 * roll_angle) + (k2 * roll_rate) + k3 * (encoder_angle - desired_steer);
  if (desiredSteerRate > 10) {
    desiredSteerRate = 10;
  }
  else if (desiredSteerRate < -10) {
    desiredSteerRate = -10;
  }
  return desiredSteerRate;
}


/*
   From the old PID.cpp File
*/
float Front_Motor_Controller::PID_Controller(float desired_pos, signed int x, signed int x_offset,
    unsigned long current_t, unsigned long previous_t, signed int oldPosition) {

  //when y value changes (when wheel passes index tick) print absolute position of the wheel now to see if encoder absolute position
  //is drifting
  float current_pos = (((x - x_offset) * 0.02197 * M_PI) / 180); //Angle (rad)

  //write PID controller based off of error signal received from encoder
  //P term
  //calculate position error (rad)
  float pos_error = desired_pos - current_pos ;

  //scaled positional error
  //position scaling factor K_p = 100/(M_PI/2) found by taking 100 (100 being max pwm value I want to reach), and dividing by theoretical max absolute value of angle (Pi/2). This means with angles in that range, 100 will be the max PWM value outputted to the motor
  float sp_error =  (K_p * pos_error);

  //D term
  //calculate velocity error
  //  unsigned long current_t = micros();
  float current_vel = (((((x - x_offset) - oldPosition) * 0.02197 * 1000000 * M_PI / 180.0) / (current_t - previous_t))); //Angular Speed(rad/s)

  //calculate the value of the current time step in microseconds
  //unsigned long delta_t = 2000;

  // the value of the velocity error will be negative of the current velocity (in order to resist current direction of motion). Calculated as target_velocity - current_velocity where target velocity is always 0
  //scaled velocity error
  float sv_error =  (-K_d * current_vel)  ;
  float total_error =  sp_error + sv_error ;

  //print total error to get a sense of how high the values are for a normal sine wave.

  if (total_error > 0) {
    digitalWrite(DIR, LOW);
  } else {
    digitalWrite(DIR, HIGH);
  }

  //clip the maximum output to the motor by essentially saying "if the value is greater than this threshold, make the output to the motor this exact threshold value"
  //  Serial.println(String(current_pos) + "\t" + String(desired_pos) + "\t" + String(pos_error) + "\t" + String(total_error));

  oldPosition = x - x_offset;
  if (total_error > 100 || total_error < -100) {
    analogWrite(PWM_front, 100);
  } else {
    analogWrite(PWM_front, abs((int)(total_error)));
  }
  return current_vel;
}

