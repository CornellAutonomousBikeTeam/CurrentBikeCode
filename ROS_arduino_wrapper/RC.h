#ifndef RC_h
#define RC_h


/*Define definite variables*/
//RC
#define RC_CH1 51     //Steer Angle 
#define RC_CH2 28     //Velocity
#define RC_CH3 25     //NOT IN USE
#define RC_CH4 33     //NOT IN USE
#define RC_CH5 27     //Nav mode
#define RC_CH6 32     //Landing Gear 
extern volatile unsigned long timer_start; //micros when the pin goes HIGH
extern volatile unsigned long timer_start2; //micros when the pin goes HIGH
extern volatile unsigned long timer_start5; //micros when the pin goes HIGH
extern volatile unsigned long timer_start6; //micros when the pin goes HIGH
extern volatile int last_interrupt_time; //calcSignal1 is the interrupt handler
extern volatile int last_interrupt_time2; //calcSignal2 is the interrupt handler
extern volatile int last_interrupt_time5; //calcSignal5 is the interrupt handler
extern volatile int last_interrupt_time6; //calcSignal6 is the interrupt handler
extern volatile float steer_range; //Used to map signal received to a steer angle
extern volatile float foreward_speed; //Used to map signal received to a rear wheel speed
extern volatile float pulse_time; //Used in calcSignal1
extern volatile float pulse_time2; //Used in calcSignal2
extern volatile float pulse_time5; //Used in calcSignal5
extern volatile float pulse_time6; //Used in calcSignal6
extern float desired_angle;  //NOT IN USE
extern int PWM_rear_output;  //NOT IN USE
extern bool nav_mode; //Nav or RC mode

/*Define functions*/
void calcSignal();

void calcSignal2();

void calcSignal5();

void calcSignal6();


#endif //RC_h

