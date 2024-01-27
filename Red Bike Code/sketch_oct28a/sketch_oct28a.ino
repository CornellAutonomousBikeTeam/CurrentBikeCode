// Uses two orange LEDS plus resistors on pins 4 and 7 (on the Nano 33 IoT; change for other boards).
// Don't pull more than 7mA from any pin.

#include <SAMD21turboPWM.h>

TurboPWM pwm;

void setup() {
  pwm.setClockDivider(1, false);
  pwm.timer(0, 1, 1000, true);
  pwm.enable(0, true);
}

void loop() {
  pwm.analogWrite(5, HIGH);
  delay(1000);
  pwm.analogWrite(5, LOW);
  delay(1000);
}

