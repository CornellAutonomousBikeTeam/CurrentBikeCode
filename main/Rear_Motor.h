#ifndef Rear_Motor_h
#define Rear_Motor_h

#include <SPI.h>
class Rear_Motor_Controller {
  
  private:
    float pwm;
    float Tnew;
    float Told;
    float dT; //Delta T
    int update_delay;

  public:
    float speed;
    float k1; //kp constant for proportional controller term

    //Constructors for Rear_Motor_Controllers
    Rear_Motor_Controller(float);
    Rear_Motor_Controller(float,int);
    
    void updateSpeed();
    void rampPWM(float);
    float getSpeed();
    void controlSpeed(float);
    void switchDirection(boolean);
    void setPins(); //Maybe change to setup
};

#endif //Rear_Motor_h
