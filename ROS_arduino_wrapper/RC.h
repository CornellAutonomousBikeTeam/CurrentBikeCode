#ifndef RC_h
#define RC_h


/*Define definite variables*/
//RC
#define RC_CH1 51     //Steer Angle 
#define RC_CH2 28     //
#define RC_CH3 25     //Velocity 
#define RC_CH4 33     //
#define RC_CH5 27     //Nav mode
#define RC_CH6 32     //Landing Gear 
extern volatile unsigned long timer_start;  //micros when the pin goes HIGH
extern volatile unsigned long timer_start2;  //micros when the pin goes HIGH
extern volatile unsigned long timer_start5; 
extern volatile unsigned long timer_start6;  //micros when the pin goes HIGH
extern volatile int last_interrupt_time; //calcSignal is the interrupt handler
extern volatile int last_interrupt_time2; //calcSignal is the interrupt handler
extern volatile int last_interrupt_time5;
extern volatile int last_interrupt_time6; //calcSignal is the interrupt handler
extern volatile float steer_range ;
extern volatile float foreward_speed ;
extern volatile float pulse_time ;
extern volatile float pulse_time2 ;
extern volatile float pulse_time5 ;
extern volatile float pulse_time6 ;
extern float desired_angle;  //CH1
extern int PWM_rear_output;  //CH3
extern bool nav_mode;

/*Define functions*/
void calcSignal();

void calcSignal2();

void calcSignal5();

void calcSignal6();


#endif //RC_h

