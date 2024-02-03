#include "ControlEquation.h"

//Global variable space
/*
You want to make the object here but initialize it with a bunch of zeros
*/
ControlEquation control_eq(0,0,0); 
//new ControlEquation(0, 0, 0) control_eq;

void setup()
{
    Serial.begin(9600);
}
void loop() 
{ 
    //ControlEquation.instance;
    control_eq.phiDot = 0.5;
    //rollangleoffset
    control_eq.rollAngleOffset = .3;
    //steerangleoffset
    control_eq.steerAngleOffset = .7;
    float result = control_eq.rollAngleAcceleration(control_eq.phiDot, control_eq.rollAngleOffset, control_eq.steerAngleOffset);
    Serial.println(result);

    delay(500); 
}
