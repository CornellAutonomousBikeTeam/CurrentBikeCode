#ifndef Bike_State_h
#define Bike_State_h

#include "Rear_Motor.h"
#include "Front_Motor.h"
#include "RC_Handler.h"
#include "Landing_Gear.h"
#include "Bike_State.h"
#include "IMU.h"
#include "Watchdog.h"

class Bike_State {
  public:
    Bike_State();

    float desired_velocity;
    float current_velocity;
    float lean_angle;
    float lean_rate;
    float encoder_position;
    float time;
    float desired_steer;

    Rear_Motor_Controller *rear;
    Front_Motor_Controller *front;
    RC_Handler rc;
    Watchdog doggo;

    

    /*
     * Helper method that is used in processLoop()
     */
    void updateIMUData();
    /*
     * Runs one iteration of main loop code from RC Bike
     */
    void processLoop();
    /*
     * Sets up all pins and such, except for interrupts
     */
    void setupBike();
};

#endif //Bike_State_h
