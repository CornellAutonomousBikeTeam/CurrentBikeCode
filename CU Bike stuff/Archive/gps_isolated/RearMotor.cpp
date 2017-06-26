#include "RearMotor.h"
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



