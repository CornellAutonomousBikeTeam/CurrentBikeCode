#include IMU_Header.h
#include "Arduino.h"

class Integrated_Bike
{
public:
private:
  Integrated_Bike(StabilityCalc sc, Wheel_Speed_Module ws, IMU imu)
  {                              // Note that Vision modules not included yet
    float leanAngle = 1;         // note: adjust when vision code available
    float steeringAngle = 1;     // note: adjust when vision code available
    float steeringAngleRate = 1; // note: adjust when vision code available
    float velocity = ws.checkFrontPWM();
    int16_t motion_equation = sc.calculateMotionEqn(leanAngle, steeringAngle, steeringAngleRate, velocity);
    // motion_equation should be turned to Navigation/Vision
    imu.IMUClasssetup();
    float roll_yaw_pitch = imu.IMUClassloop(); // return roll, yaw, pitch to Navigation/Vision
  }
}