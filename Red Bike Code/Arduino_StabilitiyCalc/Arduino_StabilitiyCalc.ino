    // put your setup code here, to run once:

  // IMPORTANT: Replace b, bikeHeight, and bikeLength with proper values
  // leanAngle = lean angle [rad]
  // steeringAngle = steering angle [rad]
  // ̇steeringAngularRate = steering angular rate [rad/s]
  // velocity = velocity [m/s]
  // b = distance from ground contact point of rear wheel to COM projected onto ground [m]
  // bikeHeight = height of the bicycle COM [m]
  // bikeLength = distance between front wheel and rear wheel ground contact point [m]

class StabilityCalc
{
  private:
    const float gravity = 9.8;  // gravitational constant
    const float bikeHeight = 0.01; 
    const float bikeLength = 0.02; 
    const float b = 0.03;

  public:
    int16_t fixgravity;
    int16_t fixbikeHeight;
    int16_t fixbikeLength;
    int16_t fixb;
    int16_t fixbikeHL;

    float final_control_val;

    StabilityCalc(){
      fixgravity = floattofixed(gravity);
      fixbikeHeight = floattofixed(bikeHeight);
      fixbikeLength = floattofixed(bikeLength);
      fixb = floattofixed(b);
      fixbikeHL= multiply_fixed(fixbikeHeight, fixbikeLength);
    } 

    int16_t floattofixed(float x){
    return (int16_t)(x * 16);
    }

    float fixedtofloat(int16_t x){
    return (x)/16.0;
    }

    int16_t multiply_fixed(int16_t d, int16_t c){
    return (int16_t) ((((int32_t) d) * ((int32_t) c)) >> 4);
    }

    int16_t divide_fixed(int16_t d, int16_t c){
    return (int16_t) ((((int32_t) d) / ((int32_t) c)) >> 4);
    }

    int16_t calculateLeanAngle(float leanAngle) {
      int16_t fixleanAngle = floattofixed(leanAngle);
      int16_t fixleangrav =  multiply_fixed(fixleanAngle, fixgravity);
      return divide_fixed(fixleangrav, fixbikeHeight);
    }

    int16_t calculateStability_Factor(float velocity, float steeringAngle){
      int16_t fixvelocity = floattofixed(velocity);
      int16_t fixsteeringAngle = floattofixed(steeringAngle);
      int16_t fixvelsquare = multiply_fixed(fixvelocity, fixvelocity);
      int16_t fixvelsteer = multiply_fixed(fixvelsquare, fixsteeringAngle);
      return divide_fixed(fixvelsteer, fixbikeHL);
    }

    int16_t calculateStability_AngularFactor(float velocity, float steeringAngularRate) {
      int16_t fixvelocity = floattofixed(velocity);
      int16_t fixvelb = multiply_fixed(fixvelocity, fixb);
      int16_t fixsteeringAngularRate = floattofixed(steeringAngularRate);
      int16_t fixvelbAngle = multiply_fixed(fixvelb, fixsteeringAngularRate);
      return divide_fixed(fixvelbAngle, fixbikeHL);
    }


    int16_t calculateMotionEqn(float leanAngleCalc, float steeringAngleCalc, float steeringAngularRateCalc, float velocitycalc){ 
     return calculateLeanAngle(leanAngleCalc) - calculateStability_Factor(velocitycalc, steeringAngleCalc) - calculateStability_AngularFactor(velocitycalc, steeringAngularRateCalc);
    }
};
    

  
  

  



 


  


