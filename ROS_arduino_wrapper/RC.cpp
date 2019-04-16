#include "RC.h"
#include <Arduino.h>

/*Variables*/
volatile unsigned long timer_start;  
volatile unsigned long timer_start2;  
volatile unsigned long timer_start5;  
volatile unsigned long timer_start6;  
volatile int last_interrupt_time; 
volatile int last_interrupt_time2;
volatile int last_interrupt_time5; 
volatile int last_interrupt_time6; 
volatile float steer_range ; 
volatile float forward_speed ; 
volatile float pulse_time ; 
volatile float pulse_time2 ; 
volatile float pulse_time5 ; 
volatile float pulse_time6 ; 
bool nav_mode;


//difference between timer_start and micros() is the length of time that the pin
//was HIGH - the PWM pulse length. volatile int pulse_time;
//this is the time that the last interrupt occurred. 
//you can use this to determine if your receiver has a signal or not.
void calcSignal()
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(RC_CH1) == HIGH)
  {
    timer_start = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start != 0 && ((volatile int)micros() - timer_start > 1100) && ((volatile int)micros() - timer_start < 1900) )
    {
      //record the pulse time
      pulse_time = ((volatile int)micros() - timer_start); //pulse time is the output from the rc value that we need to transform into a pwm value
      /*Serial.print("pulse_time ");
      Serial.println(pulse_time);*/
      //restart the timer
      timer_start = 0;
    }
  }
}

void calcSignal2()
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time2 = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(RC_CH2) == HIGH)
  {
    timer_start2 = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start2 != 0 && ((volatile int)micros() - timer_start2 > 1100) && ((volatile int)micros() - timer_start2 < 1900) )
    {
      //record the pulse time
      pulse_time2 = ((volatile int)micros() - timer_start2); //pulse time is the output from the rc value that we need to transform into a pwm value
      /*Serial.print("pulse_time2 ");
      Serial.println(pulse_time2);*/
      //restart the timer
      timer_start2 = 0;
    }
  }
}

void calcSignal5()
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time5 = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(RC_CH5) == HIGH)
  {
    timer_start5 = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start5 != 0 && ((volatile int)micros() - timer_start5 > 1000) && ((volatile int)micros() - timer_start5 < 2000) ) // filter out noise
    {
      //record the pulse time
      pulse_time5 = ((volatile int)micros() - timer_start5); //pulse time is the output from the rc value that we need to transform into a pwm value
      //restart the timer
      /*Serial.print("pulse_time5 ");
      Serial.println(pulse_time5);*/
      timer_start5 = 0;
    }
  }
  
}

void calcSignal6() 
{
  //record the interrupt time so that we can tell if the receiver has a signal from the transmitter
  last_interrupt_time6 = micros();
  //if the pin has gone HIGH, record the microseconds since the Arduino started up
  if (digitalRead(RC_CH6) == HIGH)
  {
    timer_start6 = micros();
  }
  //otherwise, the pin has gone LOW
  else
  {
    //only worry about this if the timer has actually started
    if (timer_start6 != 0 && ((volatile int)micros() - timer_start6 > 1000) && ((volatile int)micros() - timer_start6 < 2000) )
    {
      //record the pulse time
      pulse_time6 = ((volatile int)micros() - timer_start6); //pulse time is the output from the rc value that we need to transform into a pwm value
      //Serial.print("pulse_time6 ");
      //Serial.println(pulse_time6);
      //restart the timer
      timer_start6 = 0;
    }
  }
  if (pulse_time6 > 1400 && pulse_time6 < 1600) {
    nav_mode = false;
  }
  else {
    nav_mode = true;
  
  }
}
