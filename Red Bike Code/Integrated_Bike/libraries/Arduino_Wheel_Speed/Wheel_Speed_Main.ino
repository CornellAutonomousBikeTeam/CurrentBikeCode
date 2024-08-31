#include "Wheel_Speed_Module.h"

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
