#ifndef Watchdog_h
#define Watchdog_h

#include <SPI.h>

class Watchdog{ // I made this a class as I believe it may
    // be expanded in future
  private:
    //Timed Loop Variables
    const long interval = 10000;
    long l_start;
    long l_diff;
  public:
    void recordStartTime();
    void verifyEndTime();
};
#endif //Watchdog_h
