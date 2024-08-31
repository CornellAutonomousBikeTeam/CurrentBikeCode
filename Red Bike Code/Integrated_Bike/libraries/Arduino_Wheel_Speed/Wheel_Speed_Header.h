#ifndef WHEEL_SPEED_MODULE_H
#define WHEEL_SPEED_MODULE_H

#include <Arduino.h>
#include <Timers.h>
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
    float frontWheelSpeed;
    float rearWheelSpeed;
    static constexpr float pulsesPerRevolution = 20.0;

    inline void resetTimer(unsigned long *timerStart);
    inline unsigned long readTimer(unsigned long *timerStart);
    static void checkRearPWMWrapper();
    static void checkFrontPWMWrapper();
    void checkRearPWM();
    void checkFrontPWM();
    float calculateSpeed(unsigned long period);

public:
    static Wheel_Speed_Module *instance;
    Wheel_Speed_Module(byte frontPWM, byte rearPWM, byte reverse);
    void begin(unsigned long baudRate = 1000000);
    float getFrontWheelSpeed() const;
    float getRearWheelSpeed() const;
};

#endif
