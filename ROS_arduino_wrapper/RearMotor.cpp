#include "RearMotor.h"

/*Variables*/

float rear_pwm = 0; 
double speed = 0; 
boolean forward = true; 
float tOld = 0; 
float tNew = 0;
double T = 0; 

float gain_p = 5;
float desired_speed = 3; 


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
  double T = (tNew - tOld);
  if ((1.2446) * (1E6) / (28 * T) < 100) {
    //1.2446 is the Circumfrence of the wheel in meters
    //multiplying the denominator by 28 (for the 28 hall sensors), gives us a denominator of s/rev (not s/ (1/28*rev))
    speed = (1.2446) * (1E6) / (28 * T) ;
  }
}
