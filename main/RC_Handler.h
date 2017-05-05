#ifndef RC_Handler_h
#define RC_Handler_h

#include <SPI.h>
#include <math.h>
#include "Landing_Gear.h"

class RC_Handler {

  private:
    //timers for each channel
    int duration_CH1, duration_CH2, duration_CH3, duration_CH4, duration_CH5, duration_CH6;
    int start_CH1, start_CH2, start_CH3, start_CH4, start_CH5, start_CH6;
    int end_CH1, end_CH2, end_CH3, end_CH4, end_CH5, end_CH6;
    //current cycle's logic
    bool CH1, CH2, CH3, CH4, CH5, CH6;

    //RC variables
    float desired_angle;  //CH1
    int PWM_rear_output;  //CH3

    //set up timer for rc communication for steer and back motor speed
    volatile unsigned long timer_start;  //micros when the pin goes HIGH
    volatile unsigned long timer_start2;  //micros when the pin goes HIGH
    volatile unsigned long timer_start6;  //micros when the pin goes HIGH
    volatile int last_interrupt_time; //calcSignal is the interrupt handler
    volatile int last_interrupt_time2; //calcSignal is the interrupt handler
    volatile int last_interrupt_time6; //calcSignal is the interrupt handler

    
  public:
    void RCsetup();
    void calcSignal();
    void calcSignal2();
    /*
       For controlling Landing Gear
    */
    void calcSignal6();

    //TODO These ought to be private
    volatile float steer_range ;
    volatile float foreward_speed ;
    volatile float pulse_time ;
    volatile float pulse_time2 ;
    volatile float pulse_time6 ;
    volatile float old_pulse_time6 = 1500 ; //value to store previous pulse length for comparrison, 1500 corresponds to RC_CH6 set to 1 (middle) which means do nothing in terms of landing gear

};
#endif //RC_Handler_h
