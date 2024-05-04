#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>

class Timer 
{
  private:
    //private members 
    unsigned long timerStart;

  public:
    //public meethods 
    Timer();
    void reset1();
    void read1();
    unsigned long read();
    void checkREARTIMER();
    void checkFRONTPWM()
};

