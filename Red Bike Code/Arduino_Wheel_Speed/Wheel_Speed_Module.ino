class Wheel_Speed_Module
{
private:
  const byte frontPWM;
  unsigned long frontPWMTimer;
  const byte rearPWM;
  const byte rearTimer = 4;
  unsigned long rearTimer_Timer;
  const byte reverse;
  const byte I_2 = 1;
  const byte I_1 = 2;

  inline void resetTimer(unsigned long *timerStart)
  {
    *timerStart = micros();
  }

  inline void readTimer(unsigned long *timerStart)
  {
    unsigned long currentTime = micros();
    if (currentTime > *timerStart)
    {
      unsigned long elapsedTime = currentTime - *timerStart;
      Serial.println(elapsedTime);
    }
  }

  static void checkRearPWMWrapper()
  {
    // Wrapper function to call the member function from the interrupt
    instance->checkRearPWM();
  }

  static void checkFrontPWMWrapper()
  {
    // Wrapper function to call the member function from the interrupt
    instance->checkFrontPWM();
  }

  void checkRearPWM()
  {
    readTimer(&rearTimer_Timer);
    resetTimer(&rearTimer_Timer);
  }

  void checkFrontPWM()
  {
    if (digitalRead(frontPWM))
      resetTimer(&frontPWMTimer);
    else
      readTimer(&frontPWMTimer);
  }

public:
  static Wheel_Speed_Module *instance;
  Wheel_Speed_Module(byte frontPWM, byte rearPWM, byte reverse)
      : frontPWM(frontPWM), rearPWM(rearPWM), reverse(reverse)
  {
    pinMode(rearTimer, INPUT);
    pinMode(frontPWM, INPUT);

    attachInterrupt(digitalPinToInterrupt(rearTimer), checkRearPWMWrapper, CHANGE);
    attachInterrupt(digitalPinToInterrupt(frontPWM), checkFrontPWMWrapper, CHANGE);

    resetTimer(&frontPWMTimer);
    resetTimer(&rearTimer);
  }

  void begin(unsigned long baudRate = 1000000)
  {
    Serial.begin(baudRate);
  }
};

// Define the static instance pointer
Wheel_Speed_Module *Wheel_Speed_Module::instance = nullptr;

void setup()
{
  // Instantiate the Wheel_Speed_Module class
  Wheel_Speed_Module::instance = new Wheel_Speed_Module(0, 3, 5);
  Wheel_Speed_Module::instance->begin();
}

void loop()
{
  delay(10);
}
