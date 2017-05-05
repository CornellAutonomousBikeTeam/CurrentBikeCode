#include "Landing_Gear.h"

#define relay1 48
#define relay2 47
#define relay3 50
#define relay4 49

//TODO why are you guys using different relays in setup than in methods?
void landingGearSetup() {
  //pinMode(relay1, OUTPUT);
  //pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
}
void landingGearDown() {
  Serial.println("drop it low gurl");
  digitalWrite(relay1, LOW); //sets relay pin 1 to High (turns light on)
  digitalWrite(relay2, LOW); //sets relay pin 2 to High  (turns light on)
}
void landingGearUp() {
  digitalWrite(relay1, HIGH); //Sets relay pin 1 to low (turns light off)
  digitalWrite(relay2, HIGH); //Sets relay pin 2 to low (turns light off)
}
