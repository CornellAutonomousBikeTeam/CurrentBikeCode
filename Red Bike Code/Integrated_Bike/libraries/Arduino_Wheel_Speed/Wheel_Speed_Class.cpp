#include "Wheel_Speed_Module.h"

// Define the static instance pointer
Wheel_Speed_Module *Wheel_Speed_Module::instance = nullptr;

Wheel_Speed_Module::Wheel_Speed_Module(byte frontPWM, byte rearPWM, byte reverse)
    : frontPWM(frontPWM), rearPWM(rearPWM), reverse(reverse)
{
    pinMode(rearTimer, INPUT);
    pinMode(frontPWM, INPUT);

    attachInterrupt(digitalPinToInterrupt(rearTimer), checkRearPWMWrapper, CHANGE);
    attachInterrupt(digitalPinToInterrupt(frontPWM), checkFrontPWMWrapper, CHANGE);

    resetTimer(&frontPWMTimer);
    resetTimer(&rearTimer_Timer);
}

void Wheel_Speed_Module::begin(unsigned long baudRate)
{
    Serial.begin(baudRate);
}

inline void Wheel_Speed_Module::resetTimer(unsigned long *timerStart)
{
    *timerStart = micros();
}

inline unsigned long Wheel_Speed_Module::readTimer(unsigned long *timerStart)
{
    unsigned long currentTime = micros();
    if (currentTime > *timerStart)
    {
        return currentTime - *timerStart;
    }
    return 0;
}

void Wheel_Speed_Module::checkRearPWMWrapper()
{
    instance->checkRearPWM();
}

void Wheel_Speed_Module::checkFrontPWMWrapper()
{
    instance->checkFrontPWM();
}

void Wheel_Speed_Module::checkRearPWM()
{
    unsigned long period = readTimer(&rearTimer_Timer);
    resetTimer(&rearTimer_Timer);

    if (period > 0)
    {
        rearWheelSpeed = calculateSpeed(period);
        Serial.print("Rear Wheel Speed: ");
        Serial.println(rearWheelSpeed);
    }
}

void Wheel_Speed_Module::checkFrontPWM()
{
    unsigned long period = readTimer(&frontPWMTimer);
    resetTimer(&frontPWMTimer);

    if (period > 0)
    {
        frontWheelSpeed = calculateSpeed(period);
        Serial.print("Front Wheel Speed: ");
        Serial.println(frontWheelSpeed);
    }
}

float Wheel_Speed_Module::calculateSpeed(unsigned long period)
{
    if (period > 0)
    {
        return 60.0 * 1000000.0 / (period * pulsesPerRevolution);
    }
    return 0.0;
}

float Wheel_Speed_Module::getFrontWheelSpeed() const
{
    return frontWheelSpeed;
}

float Wheel_Speed_Module::getRearWheelSpeed() const
{
    return rearWheelSpeed;
}
