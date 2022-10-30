#include "LandingGear.h"
#include <Arduino.h>

/*Variables*/
float front_steer_value ;
float back_wheel_speed ;
float steer_contribution ;
float commanded_speed ;

/*Functions*/
void landingGearDown() {
  digitalWrite(48, LOW); //sets relay pin 1 to High (turns light on)
  digitalWrite(47, LOW); //sets relay pin 2 to High  (turns light on)
}
void landingGearUp() {
  digitalWrite(48, HIGH); //Sets relay pin 1 to low (turns light off)
  digitalWrite(47, HIGH); //Sets relay pin 2 to low (turns light off)
}

void initLandingGear() {
  pinMode(LG_RELAY_1, OUTPUT);
  pinMode(LG_RELAY_2, OUTPUT);
}

void setLandingGear(int state) {
  digitalWrite(LG_RELAY_1, state);
  digitalWrite(LG_RELAY_2, state);
}
