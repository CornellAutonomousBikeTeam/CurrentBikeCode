#ifndef LandingGear_h
#define LandingGear_h


/*Define definite variables*/
#define LG_RELAY_1 48
#define LG_RELAY_2 47
extern float front_steer_value ;
extern float back_wheel_speed ;
extern float steer_contribution ;
extern float commanded_speed ;

// relays HIGH = light on = gear down
// relays LOW = light off = gear up
#define LG_UP HIGH
#define LG_DOWN LOW

void initLandingGear();
void setLandingGear(int);

#endif

