#ifndef ControlEquation_H
#define ControlEquation_H

#include <cmath> 

class ControlEquation
{
    private:
        const float gravity = 9.8;  // gravitational constant
        const float bikeHeight = 1; // wheelbase height 
        const float bikeLength = 1; // wheelbase length 
        const float b = 1; // distance between the center of mass and the rear axle
        const float k1 = 1; // gain from roll angle 
        const float k2 = 1; // gain from roll angular velocity 
        const float k3 = 1; // gain from steering angle 
        
    public:
        float fixgravity = 1;
        float fixbikeHeight = 1;
        float fixbikeLength = 1;
        float fixb = 1;
        float fixbikeHL = 1;
        float phiDot = 1;
        float rollAngleOffset = 1;
        float steerAngleOffset = 1;

        // Public Methods
        ControlEquation(float phi_dot, float rollAngleOffset, float steerAngleOffset);
        float multiply_fixed(float a, float b);
        float divide_fixed(float a, float b);
        float controlVariableCalculation(float rollAngle, float rollAngleOffset, float steeringAngle, float steeringAngleOffset);
        float xPosDeriv(float velocity, float psi);
        float yPosDeriv(float velocity, float psi);
        float rollAngleVelocity();
        float yawAngleVelocity(float velocity, float steerAngle, float rollAngle);
        float velocityDeriv();
        float rollAngleAcceleration(float rollAngle, float steerAngle, float velocity);
};
#endif //ControlEquation_H