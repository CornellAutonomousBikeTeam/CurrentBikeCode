#ifndef BalanceController_h
#define BalanceController_h


//Balance Control constants
#define k1 70 //phi = lean
#define k2 10 //was previously 21 //phidot=lean rate
#define k3 -20 //delta=steer

/*Define functions*/
//PID
float balanceController(float roll_angle, float roll_rate,
                        float encoder_angle, float desired_steer);


#endif //PID_h

//Gameplan:
//Try out different gains off ground for holding position (stationary testing)
//Try out different gains ON ground (max 45 minutes) (step input)
//if failed, try different gains OFF ground, tune (step input)

