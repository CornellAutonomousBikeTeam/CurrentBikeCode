#include "IMU_Header.h"
#include "Arduino.h"

// Constructor
IMU::IMU() : CSN(1), SO(10), SI(8), CLK(9), delta(180 / 3.14), INST(0x01), instructions{0x01, 0x02, 0x04, 0x08}, flinst{3, 9, 6, 9},
             settings(6000000, MSBFIRST, SPI_MODE0), instruction_number(0), startMillis(0), currentMillis(0) {}

// Method Implementations
void IMU::IMUClasssetup()
{
    pinMode(CSN, OUTPUT);
    pinMode(SI, OUTPUT);
    pinMode(SO, INPUT);
    pinMode(CLK, OUTPUT);
    digitalWrite(CSN, HIGH);
    SPI.begin();
    Serial.begin(9600);
}

byte IMU::transferByte(byte byteToWrite)
{
    byte Result = 0x00;
    digitalWrite(CSN, LOW);
    delay(1);
    Result = SPI.transfer(byteToWrite);
    delay(1);
    digitalWrite(CSN, HIGH);
    return Result;
}

void IMU::endianSwap(byte temp[4])
{
    byte myTemp = temp[0];
    temp[0] = temp[3];
    temp[3] = myTemp;
    myTemp = temp[1];
    temp[1] = temp[2];
    temp[2] = myTemp;
}

byte IMU::readData(byte instruction)
{
    byte result = transferByte(0x01);
    // Serial.print("Cleared internal buffer. Result: "), Serial.println(result);

    // Send start of packet:
    result = transferByte(0xF6);
    // Serial.print("Send start of packet. Result: "), Serial.println(result);

    // Send command (tared euler angles)
    result = transferByte(0x07);
    // Serial.print("Send commmand 0x01. Result: "), Serial.println(result); // what this line does

    // Get status of device:
    result = transferByte(0xFF);
    // Serial.print("Status of device. Result: "), Serial.println(result);
}

float *IMU::IMUClassloop()
{
    SPI.beginTransaction(settings);
    byte result_loop = readData(instructions[instruction_number]);

    while (result_loop != 0x01)
    {
        delay(1);
        result_loop = transferByte(0xFF);
        // Serial.println(result_loop);
    }

    for (int i = 0; i < (flinst[instruction_number]); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            data[i].b[j] = transferByte(0xFF);
            delay(1);
        }
    }

    SPI.endTransaction();

    if (result_loop == 0x01)
    {
        for (int mm = 0; mm < 3; mm++)
        {
            endianSwap(data[mm].b);
        }

        startMillis = micros();
        data[0].fval *= delta;
        Serial.println(data[0].fval);
        data[1].fval *= delta;
        Serial.println(data[1].fval);
        data[2].fval *= delta;
        Serial.println(data[2].fval);
        currentMillis = micros();

        // Yanjun: We are running into code limitations as we are trying to keep
        // this variable static but also access it outside.
        // Having problems keeping it updating so using Jank solution for now
        static float degsval[3] = {data[0].fval, data[1].fval, data[2].fval};

        return degsval;
    }
    delay(3000);
    return nullptr;
}
