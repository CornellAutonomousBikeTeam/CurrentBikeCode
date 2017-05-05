#ifndef Landing_Gear_h
#define Landing_Gear_h

#include <SPI.h>

class Landing_Gear { //I didn't realize that half this stuff was not used anywhere...
  private:

    float front_steer_value ;
    float back_wheel_speed ;
    float steer_contribution ;
    float commanded_speed ;


};
void landingGearSetup();
void landingGearDown();
void landingGearUp();
#endif //Landing_Gear_h
