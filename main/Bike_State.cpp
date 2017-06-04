#include "Bike_State.h"
#include "IMU.h"
#include "SPI.h"



//Rear Motor
#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63 // Battery Voltage pin
#define reverse_pin 50

//Front Motor
#define PWM_front 9
#define DIR 46

//RC
#define RC_CH1 51     //Steer Angle 
#define RC_CH2 28     //
#define RC_CH3 25     //Velocity 
#define RC_CH4 33     //
#define RC_CH5 27     //Kill Switch 
#define RC_CH6 32     //Landing Gear 

Bike_State::Bike_State(){
    delete rear;
    rear = new Rear_Motor_Controller(8);
    delete front;
    front = new Front_Motor_Controller(2000, -20, 0);
}

void Bike_State::updateIMUData() {
  lean_angle = getIMU(0x01);
  lean_rate = getIMU(0x26);
}
void Bike_State::setupBike() {
  
  Serial.println(2);
  
  rear->setPins();
  Serial.println(3);
  rc.RCsetup();
  Serial.println(4);
  landingGearSetup();
  Serial.println(5);
  front->encoderAndFrontMotorSetup();
  Serial.println(6);
  initIMU();
  Serial.println(7);
  
  rear->rampPWM(90);
  Serial.println(8);
  front->calibrate();
  Serial.println(9);
}
void Bike_State::processLoop() {
  //rear.controlSpeed(1.5); //TODO This somehow affects front motor calibration
  //TODO figure out why calibration is unstable- I noticed it broke after I accidentally uploaded
  //or reopened serial monitor with front motor on (ie when if flips out).
  Serial.println(rear->speed);
  //Serial.println(state.lean_angle);
  //Serial.println(state.lean_rate);

  //========================== STUFF IDK WHAT BE=======================
  //first trial of rc stuff, trying to get steer angle in RC_CH1 to read in rc input
  //
  rc.foreward_speed = map(rc.pulse_time2, 1100, 1900, 0, 200);

  analogWrite(pwm_rear, rc.foreward_speed);
  rc.steer_range = map(rc.pulse_time, 1100, 1900, -70, 70);

  front->desired_steer = rc.steer_range * .01 ;
  //=================================================================
  doggo.recordStartTime();

  float encoder_position = front->updateEncoderPosition();
  updateIMUData();


  /*
     TODO Figure out which velocity the following variables deal with
  */
  //Serial.println(String(encoder_position) + '\t' + String(imu_data.angle) + '\t' + String(imu_data.rate)) ;
  //int imu_data_angle = 0; int imu_data_rate = 0; // ***IF YOU USE THIS LINE YOU MUST CHANGE THE FOLLOWING LINE TO imu_data_rate and ime_data_angle because those are the correct variable names
  desired_velocity = front->balanceController(((1) * (lean_angle)), (1) * lean_rate, encoder_position); //*****PUT IN OFFSET VALUE BECAUSE THE IMU IS READING AN ANGLE OFF BY +.16 RADIANS
  current_velocity = front->frontWheelControl((-1) * desired_velocity, encoder_position); //DESIRED VELOCITY SET TO NEGATIVE TO MATCH SIGN CONVENTION BETWEEN BALANCE CONTROLLER AND ... cmon man

  doggo.verifyEndTime();
}

