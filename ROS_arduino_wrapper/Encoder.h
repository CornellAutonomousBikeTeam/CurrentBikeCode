#ifndef Encoder_h
#define Encoder_h
#include <Arduino.h>


//Read the relative position of the encoder
signed int relativePos = REG_TC0_CV0;
//Read the index value (Z channel) of the encoder
signed int indexValue = REG_TC0_CV1;


#endif //Encoder_h

