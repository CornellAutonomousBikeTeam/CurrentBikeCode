#include "ControlEquation.h"
#include <math.h>.
float phi_dot;
float rollAngleOffset; 
float steerAngleOffset; 

ControlEquation::ControlEquation(float phi_dot, float rollAngleOffset, float steerAngleOffset){
  phi_dot = phi_dot;
  rollAngleOffset = rollAngleOffset;
  steerAngleOffset = steerAngleOffset;

}

ControlEquation::ControlEquation(){
  phi_dot = 0;
  rollAngleOffset = 0;
  steerAngleOffset = 0;

}


//Have to fix syntax issues for method definition such that ControlEquation::multiply_fixed
float ControlEquation::multiply_fixed(float a, float b){
    return (a*b);
}

float ControlEquation::divide_fixed(float a, float b){
    return (a/b);
}

float ControlEquation::controlVariableCalculation(float rollAngle, float rollAngleOffset, float steeringAngle, float steeringAngleOffset){
    return(multiply_fixed(k1, (rollAngle - rollAngleOffset))+ multiply_fixed(k2,rollAngle)+multiply_fixed(k3, (steeringAngle - steeringAngleOffset)));
}

float ControlEquation::xPosDeriv(float velocity, float psi){
    return(multiply_fixed(velocity, cos(psi)));
}

float ControlEquation::yPosDeriv(float velocity, float psi){
    return(multiply_fixed(velocity, sin(psi)));
}

float ControlEquation::rollAngleVelocity(){
    return phiDot;
}

float ControlEquation::yawAngleVelocity(float velocity, float steerAngle, float rollAngle){
    return(multiply_fixed(divide_fixed(velocity, bikeLength), divide_fixed(tan(steerAngle), cos(rollAngle))));
}

float ControlEquation::velocityDeriv(){
    return 0;
}

float ControlEquation::rollAngleAcceleration(float rollAngle, float steerAngle, float velocity){
    // 1/h
    // phi (0/) -> roll angle
    // delta (curly) --> steering angle..
    float hReciprocal = divide_fixed(1,fixbikeHeight); 
    float gSinRollAngle = multiply_fixed(gravity, sin(rollAngle));
    float tanSteerAngle = tan(steerAngle);
    float velocitySquaredOverL = divide_fixed(multiply_fixed(velocity, velocity), bikeLength);
    float bVelocityDerivOverL = divide_fixed(multiply_fixed(fixb,velocityDeriv()), fixbikeLength);
    float tanRollAngle = tan(rollAngle);
    float bVelocityPhiDotOverL = divide_fixed((multiply_fixed(fixb,velocity), rollAngleVelocity()),bikeLength);
    float hVelocitySquaredTanOverLSquared = divide_fixed(((fixbikeHeight, multiply_fixed(velocity,velocity)), tanSteerAngle), multiply_fixed(bikeLength, bikeLength));
    float bVelocitySteerAngle = multiply_fixed(multiply_fixed(fixb,velocity), controlVariableCalculation(rollAngle,rollAngleOffset, steerAngle, steerAngleOffset));
    float lengthCosSquaredSteerAngle = multiply_fixed(bikeHeight, multiply_fixed(cos(steerAngle), cos(steerAngle)));
    float Equation = (gSinRollAngle - multiply_fixed(tanSteerAngle, (velocitySquaredOverL+bVelocityDerivOverL+multiply_fixed(tanRollAngle, (bVelocityPhiDotOverL - hVelocitySquaredTanOverLSquared)))) - divide_fixed(bVelocitySteerAngle,lengthCosSquaredSteerAngle));
    return (Equation);
}