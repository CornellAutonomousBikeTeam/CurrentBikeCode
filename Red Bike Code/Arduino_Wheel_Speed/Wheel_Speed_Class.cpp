#include "Wheel_Speed_Header.h"
#include "Arduino.h"

Timer::Timer() {
  timerStart = 0;
}

void Timer::reset() {
  timerStart = micros();
}

unsigned long Timer::read() {
  unsigned long currentTime = micros();
  if (currentTime > timerStart) {
    return currentTime - timerStart;
  }
  return 0;
}

inline void reset1(unsigned long *timerstart)
{
  *timerstart = micros();
}

inline void read1(unsigned long *timerstart)
{}

inline void reset1(unsigned long * timerstart) {
  *timerstart = micros();
}

inline void  read1(unsigned long * timerstart) {
  unsigned long currentTime1 = micros();
  if (currentTime1 > *timerstart)
  {
    unsigned long RearTimer1 = currentTime1 - *timerstart;
    Serial.println(RearTimer1); //Experimental
  }
}

void checkREARTIMER()
{
  read1(&REARTIMER_TIMER);
  reset1(&REARTIMER_TIMER);
}

void checkFRONTPWM()
{
  if (digitalRead(FRONTPWM))
    reset1(&FRONT_PWM_TIMER);
  else
    read1(&FRONT_PWM_TIMER);
}

void setup()
{
  Serial.begin(1000000);
  // Set switch pin as INPUT with pullup
  pinMode(REARTIMER, INPUT);

  pinMode(FRONTPWM, INPUT);

  // Attach Interrupt to Interrupt Service Routine
  attachInterrupt(digitalPinToInterrupt(REARTIMER), checkREARTIMER, CHANGE); // Pin to REARTIMER
  attachInterrupt(digitalPinToInterrupt(FRONTPWM), checkFRONTPWM, CHANGE);   // Pin to REARTIMER
}

void loop()
{
  delay(10);
}
