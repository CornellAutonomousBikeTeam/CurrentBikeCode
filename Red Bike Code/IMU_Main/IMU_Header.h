#ifndef IMU_H
#define IMU_H

#include <SPI.h>

class IMU
{
private:
    // Private members
    const int CSN, SO, SI, CLK, delta;
    const byte INST;
    const byte instructions[4];
    const float flinst[4];

public:
    // Public members
    SPISettings settings;
    const int instruction_number;
    unsigned long startMillis;
    unsigned long currentMillis;
    union u_types {
        byte b[4];
        float fval;
    } data[3];

    // Public Methods
    IMU();
    void IMUClasssetup();
    byte transferByte(byte byteToWrite);
    void endianSwap(byte temp[4]);
    byte readData(byte instruction);
    float* IMUClassloop();
};

#endif // IMU_H
