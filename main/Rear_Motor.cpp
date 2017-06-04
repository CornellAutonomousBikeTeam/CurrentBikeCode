#include "Rear_Motor.h"
#define in_pin 11 //hall sensor pulse 
#define pwm_rear 8 //rear motor PWM pin
#define v_pin 63 // Battery Voltage pin
#define reverse_pin 50
/*
   Constructor for Instantiating Proportional Controller
*/
Rear_Motor_Controller::Rear_Motor_Controller(float Pgain, int set_delay): k1(Pgain), update_delay(set_delay) {};

/*
   Constructor for Instantiating Proportional Controller without setting
   the delay time: default 100
*/
Rear_Motor_Controller::Rear_Motor_Controller(float Pgain): k1(Pgain), update_delay(100) {};

float Rear_Motor_Controller::getSpeed() {
  return speed;
}
/*
   Sets the speed value in the motor controller
*/
void Rear_Motor_Controller::updateSpeed() {
  float Told = Tnew;
  Tnew = micros();
  dT = (Tnew - Told);
  if ((1.2446) * (1E6) / (28 * dT)  < 100) {
    speed = (1.2446) * (1E6) / (28 * dT);
  }
}
/*
   Uses Proportional Controller to write new speed to bike rear motor. Currently
   being edited.
*/
void Rear_Motor_Controller::controlSpeed(float commanded_speed) {
  pwm = (int)(k1 * (commanded_speed - speed) + pwm);
  if (pwm > 180)
    pwm = 180;
  if (pwm < 60)
    pwm = 60;
  analogWrite(pwm_rear, pwm);
  delay(update_delay);
}
/**
   Helper method that ramps to a desired pwm value.
*/
void Rear_Motor_Controller::rampPWM(float desired_PWM) {
  while (pwm < desired_PWM) { //Ramps up speed- Workaround for rear motor safety features
    analogWrite(pwm_rear, pwm);
    delay(update_delay);
    pwm = pwm + 10;
    Serial.println(pwm);
    //Serial.print("\n");
  }
}
/*
  Switches direction of motor: Input true for forward direction, and false
  to move in reverse.
*/
void Rear_Motor_Controller::switchDirection(boolean forward) {
  // Serial.println("entered method");
  if (forward) {
    digitalWrite(reverse_pin, LOW); //when low the motor goes reverse
    boolean forward = false;
    //Serial.println("set to reverse");
  } else {
    digitalWrite(reverse_pin, HIGH); //when high the motor goes forward
    forward = true;
    // Serial.println("set to forward");
  }
}
/*
   Convenience method for setting up Rear Motor pins
*/
void Rear_Motor_Controller::setPins() {
  pinMode(in_pin, INPUT);
  pinMode(pwm_rear, OUTPUT);
}

