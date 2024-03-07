
//8-byte scale 0 - 255 (0 to full power)
unsigned long FRONT_PWM_TIMER;
const byte FRONTPWM = 0;
const byte I_2 = 1;
const byte I_1 = 2;
const byte REARPWM = 3;
const byte REARTIMER = 4;
unsigned long REARTIMER_TIMER;
const byte REVERSE = 5;


inline void reset1(unsigned long *timerstart)
{
  *timerstart = micros();
}

inline void read1(unsigned long *timerstart)
{}


// reset1 uses the encoder so it resets the timer when an active high is detected
inline void reset1(unsigned long * timerstart) {
  *timerstart = micros();
}

// read1 uses the timer to keep track of time of the wheel revolution, dependent on reset1 (resets with reset1)
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
