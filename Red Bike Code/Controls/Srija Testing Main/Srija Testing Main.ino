#include "Srija Testing Header.h"

//Global variable space
/*
You want to make the object here but initialize it with a bunch of zeros
*/
ControlEquation control_eq = new ControlEquation(0, 0, 0);

void setup()
{
    Serial.begin(9600);
}
void loop() 
{ 
    //ControlEquation.instance;
    control_eq.phi_dot = 0.5;
    //rollangleoffset
    control_eq.rollAngleOffset = .3;
    //steerangleoffset
    control_eq.steerAngleOffset = .7;
    float result = rollAngleAcceleration(control_eq.phi_dot, control_eq.rollAngleOffset, control_eq.steerAngleOffset);
    Serial.println(result);

    delay(500); 
}
