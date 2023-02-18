    // put your setup code here, to run once:

  // IMPORTANT: Replace b, bikeHeight, and bikeLength with proper values
  // leanAngle = lean angle [rad]
  // steeringAngle = steering angle [rad]
  // ̇steeringAngularRate = steering angular rate [rad/s]
  // velocity = velocity [m/s]
  // b = distance from ground contact point of rear wheel to COM projected onto ground [m]
  // bikeHeight = height of the bicycle COM [m]
  // bikeLength = distance between front wheel and rear wheel ground contact point [m]


void setup() {

}



void loop()
{
}

  const float gravity = 9.8;  // gravitational constant
  const float bikeHeight = 0.01; 
  const float bikeLength = 0.02; 
  const float b = 0.03; 
  

  float multiply(float a, float b){
      return 0;
    }
  float bikeHL = multiply(bikeHeight, bikeLength);

  float divide(float a, float b){
      return 0;
    }

  float calculateLeanAngle(float leanAngle) {
    float leangrav =  multiply(leanAngle, gravity);
    return divide(leangrav, bikeHeight);
  }




  float calculateStability_Factor(float velocity, float steeringAngle){
    float velsquare = multiply(velocity, velocity);
    float velsteer = multiply(velsquare, steeringAngle);
    return divide(velsteer, bikeHL);
  }




  float calculateStability_AngularFactor(float velocity, float steeringAngularRate) {
    float velb = multiply(velocity, b);
    float velbAngle = multiply(velb, steeringAngularRate);
    return divide(velbAngle, bikeHL);
  }

  

  float calculateMotionEqn(float leanAngleCalc, float stabilityFactorCalc, float stabilityAngularFactorCalc) {
    return calculateLeanAngle(20) - calculateStability_Factor(20, 20) - calculateStability_AngularFactor(20, 20);
  }

  


