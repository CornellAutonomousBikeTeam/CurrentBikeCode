#include "RearMotor.h"

/*Variables*/

//Rear Motor Variables
float rear_pwm = 0; //current pwm value
double speed = 0; //speed in rev/s
boolean forward = true; //if False, is running in reverse
//Variables for calculating rear motor speed
float tOld = 0; //first time reading
float tNew = 0; //second time reading

//Debugging
double hall_sensor_timestamp = 0;
int hall_sensor_tick_count = 0;
//Rear motor controller variable
float gain_p = 5;
float desired_speed = 3; //(m/s)



/*
   Method for setting rear motor at a certain PWM

   @param newPWM- the new pwm value to ramp up to
*/
void rampToPWM(float newPWM, float rear_pwm) {
  
  while (rear_pwm < newPWM) { //Ramps up speed- Workaround for rear motor safety features
    if (newPWM - rear_pwm < 10)
      rear_pwm += newPWM - rear_pwm;
    else
      rear_pwm += 10;
    analogWrite(PWM_rear, rear_pwm);
    delay(100);
  }
  while (rear_pwm > newPWM) {
    if (rear_pwm - newPWM < 10)
      rear_pwm -= rear_pwm - newPWM;
    else
      rear_pwm -= 10;
    analogWrite(PWM_rear, rear_pwm);
    delay(100);
  }
}


/*
   Switches direction of motor
*/
void switchDirection(boolean forward) {
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

void getPeriod() {
  float tOld = tNew;
  tNew = micros();
  hell_sensor_tick_count++;
  hall_sensor_timestamp = (tNew - tOld);
  if ((1.2446) * (1E6) / (28 * hall_sensor_timestamp) < 100) {
    //1.2446 is the Circumfrence of the wheel in meters
    //multiplying the denominator by 28 (for the 28 hall sensors), gives us a denominator of s/rev (not s/ (1/28*rev))
    speed = (1.2446) * (1E6) / (28 * hall_sensor_timestamp) ;
  }
}
