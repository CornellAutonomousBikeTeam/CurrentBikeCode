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
  float frontWheelSpeed;  // variable to store front wheel speed
  float rearWheelSpeed;   // variable to store reare wheel speed
  static constexpr float pulsesPerRevolution = 20.0; // Example value, adjust to your sensor's specification

  inline void resetTimer(unsigned long *timerStart)
  {
    *timerStart = micros();
  }

  inline void readTimer(unsigned long *timerStart) //changed "void" to unsigned long
  {
    unsigned long currentTime = micros();
    if (currentTime > *timerStart)
    {
      /*unsigned long elapsedTime = currentTime - *timerStart; */
      return currentTime - *timerStart;
    }  
      return 0;
      Serial.println(elapsedTime);
    
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
    //readTimer(&rearTimer_Timer);
    unsigned long period = readTimer(&rearTimer_Timer);  // Use returned period
    resetTimer(&rearTimer_Timer);

    if (period > 0)
    {
      rearWheelSpeed = calculateSpeed(period);  // Calculate rear wheel speed
      Serial.print("Rear Wheel Speed: ");
      Serial.println(rearWheelSpeed);
    }
  }

  void checkFrontPWM()
  {
    /*if (digitalRead(frontPWM))
      resetTimer(&frontPWMTimer);
    else
      readTimer(&frontPWMTimer); */
    unsigned long period = readTimer(&frontPWMTimer);  // Use returned period
    resetTimer(&frontPWMTimer);

    if (period > 0)
    {
      frontWheelSpeed = calculateSpeed(period);  // Calculate front wheel speed
      Serial.print("Front Wheel Speed: ");
      Serial.println(frontWheelSpeed);
  }


  float calculateSpeed(unsigned long period)  // New function to calculate speed
  {
    // Assuming period is in microseconds and the wheel speed needs to be in RPM
    if (period > 0)
    {
      return 60.0 * 1000000.0 / (period * pulsesPerRevolution);
    }
    return 0.0;
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

  float getFrontWheelSpeed() const { return frontWheelSpeed; }  // New accessor method for front wheel speed
  float getRearWheelSpeed() const { return rearWheelSpeed; }    // New accessor method for rear wheel speed
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
