#include "Rear_Motor.h"
#include "Front_Motor.h"
#include "RC_Handler.h"
#include "Landing_Gear.h"
#include "Bike_State.h"
#include "IMU.h"
#include "Watchdog.h"
//============================ROS=================
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;

std_msgs::Float32MultiArray bike_state;
ros::Publisher state_pub("bike_state", &bike_state);
//============================END ROS===================

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


//==========================Initialize Controllers===============
Bike_State bike;


void setup() {
  
  Serial.begin(19200);
  Serial.println(0);


  attachInterrupt(digitalPinToInterrupt(in_pin), update_Rear_Motor_Speed, RISING);

  attachInterrupt(RC_CH1, rc1, CHANGE);
  attachInterrupt(RC_CH2, rc2, CHANGE);
  attachInterrupt(RC_CH6, rc6, CHANGE);
    Serial.println(1);
  bike.setupBike();
      Serial.println(3);
}

void loop() {
  Serial.println('Top of the loop');
  bike.processLoop();

  
  bike_state.data[0] = bike.desired_velocity;
  bike_state.data[1] = bike.current_velocity;
  bike_state.data[2] = bike.lean_rate;
  bike_state.data[3] = bike.lean_angle;
  bike_state.data[4] = bike.encoder_position;
  bike_state.data[5] = bike.desired_steer;
  state_pub.publish( &bike_state );
  nh.spinOnce();
}


void update_Rear_Motor_Speed() {
  bike.rear.updateSpeed();
}
void rc1() {
  bike.rc.calcSignal();
}
void rc2() {
  bike.rc.calcSignal2();
}
void rc6() {
  bike.rc.calcSignal6();
}

