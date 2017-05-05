#include "RC_Handler.h"
//RC
#define RC_CH1 51     //Steer Angle 
#define RC_CH2 28     //
#define RC_CH3 25     //Velocity 
#define RC_CH4 33     //
#define RC_CH5 27     //Kill Switch 
#define RC_CH6 32     //Landing Gear 

//difference between timer_start and micros() is the length of time that the pin 
//was HIGH - the PWM pulse length. volatile int pulse_time; 
//this is the time that the last interrupt occurred. 
//you can use this to determine if your receiver has a signal or not. 

void RC_Handler::RCsetup(){
  pinMode(RC_CH3, INPUT);
  pinMode(RC_CH4, INPUT);
  pinMode(RC_CH5, INPUT);
  pinMode(RC_CH6, INPUT);
}

void RC_Handler::calcSignal() 
{
    //record the interrupt time so that we can tell if the receiver has a signal from the transmitter 
    last_interrupt_time = micros(); 
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(RC_CH1) == HIGH) 
    { 
        timer_start = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(timer_start != 0 && ((volatile int)micros() - timer_start > 1100) && ((volatile int)micros() - timer_start < 1900) )
        { 
            //record the pulse time
            pulse_time = ((volatile int)micros() - timer_start); //pulse time is the output from the rc value that we need to transform into a pwm value
            //restart the timer          
              timer_start = 0;
        }
    } 
} 
void RC_Handler::calcSignal2() 
{
    //record the interrupt time so that we can tell if the receiver has a signal from the transmitter 
    last_interrupt_time2 = micros(); 
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(RC_CH2) == HIGH) 
    { 
        timer_start2 = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(timer_start2 != 0 && ((volatile int)micros() - timer_start2 > 1100) && ((volatile int)micros() - timer_start2 < 1900) )
        { 
            //record the pulse time
            pulse_time2 = ((volatile int)micros() - timer_start2); //pulse time is the output from the rc value that we need to transform into a pwm value
            //restart the timer          
              timer_start2 = 0;
        }
    } 
} 


void RC_Handler::calcSignal6() //landing gear control
{
  
    
    //record the interrupt time so that we can tell if the receiver has a signal from the transmitter 
    last_interrupt_time6 = micros(); 
    //if the pin has gone HIGH, record the microseconds since the Arduino started up 
    if(digitalRead(RC_CH6) == HIGH) 
    { 
        timer_start6 = micros();
    } 
    //otherwise, the pin has gone LOW 
    else
    { 
        //only worry about this if the timer has actually started
        if(timer_start6 != 0 && ((volatile int)micros() - timer_start6 > 1000) && ((volatile int)micros() - timer_start6 < 2000) )// filter out noise
        { 
            //record the pulse time
            pulse_time6 = ((volatile int)micros() - timer_start6); //pulse time is the output from the rc value that we need to transform into a pwm value
            //restart the timer          
              timer_start6 = 0;

              //compare pulse_time6 to old_pulsetime6, if there is a significant difference RC_CH6 setting has changed. operate landing gear depending on RC_CH6 then store pulse_time6 into old_pulse_time6
              if (abs(old_pulse_time6 - pulse_time6) > 200){ //200 micro second set a tolerance for change in pulse time. theoritically the 3 possible settings are 1100, 1500, and 1900 (measured on oscilloscope)
                old_pulse_time6 = pulse_time6; //store current pulse_time6 into old_pulse_time6 for next iteration
                
                // CH6 set to 2 on RC
                  if (pulse_time6 > 1000 && pulse_time6 < 1200){
                 //   if (foreward_speed == 0){ //UNCOMMENT THIS SECTION TO MAKE LANDING GEAR DEPENDENT ON BACK MOTOR VELOCITY
                      landingGearDown();
                 //   }
                 //   else if (foreward_speed > 100){
                 //     landingGearUp();
                 //   }  
                  }
                   // CH6 set to 0 on RC
                  else if (pulse_time6 > 1800 && pulse_time6 < 2000){
                    landingGearUp();
                  }
                
              }
              
        }
    }    
    
} 
