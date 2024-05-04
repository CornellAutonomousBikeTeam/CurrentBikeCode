#include <Arduino.h>
#include "Timer.h"

const byte FRONTPWM = 0;
const byte I_2 = 1;
const byte I_1 = 2;
const byte REARPWM = 3;
const byte REARTIMER = 4;
const byte REVERSE = 5;

Timer rearTimer;
Timer frontPWM_Timer;

void checkREARTIMER() {
  unsigned long rearTime = rearTimer.read();
  Serial.println(rearTime); // Experimental
  rearTimer.reset();
}

void checkFRONTPWM() {
  if (digitalRead(FRONTPWM)) {
    frontPWM_Timer.reset();
  } else {
    unsigned long frontTime = frontPWM_Timer.read();
  }
}

void setup() {
  Serial.begin(1000000);
  pinMode(REARTIMER, INPUT);
  pinMode(FRONTPWM, INPUT);
  attachInterrupt(digitalPinToInterrupt(REARTIMER), checkREARTIMER, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FRONTPWM), checkFRONTPWM, CHANGE);
}

void loop() {
  delay(10);
}
