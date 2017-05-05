#include "Watchdog.h"

void Watchdog::recordStartTime() {
  l_start = micros();
}

void Watchdog::verifyEndTime() {
  l_diff = micros() - l_start;
  //Standardize the loop time by checking if it is currently less than the constant interval, if it is, add the differnce so the total loop time is standard
  if (l_diff < interval) {
    delayMicroseconds(interval - l_diff);
  } else {
    Serial.println("LOOP LENGTH WAS VIOLATED. LOOP TIME WAS: " + String(l_diff));
    while (true) {}
    //     	cli();
    //sleep_enable();
    //sleep_cpu(); <Alternatives to while(true)
  }
}
