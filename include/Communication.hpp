#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <iostream>
#include <stdio.h>
#include <Arduino.h>
#include "HardwareSerial.h"
#include <stdlib.h>

class Communication
{
private:
    typedef struct
    {
        uint16_t timestamp;    // 2 bytes
        int16_t positionLeft;  // 2 bytes
        int16_t positionRight; // 2 bytes
        int16_t currentLeft;   // 2 bytes
        int16_t currentRight;  // 2 bytes
        int16_t voltageLeft;   // 2 bytes
        int16_t voltageRight;  // 2 bytes
    } packetData;

    struct actuatorData
    {
        float position;
        float current;
        float voltage;
    };

    typedef struct{
        actuatorData left;
        actuatorData right;
    } wingData;

    int16_t encodeFloatToInt(float value);

    float decodeFloatToInt(int16_t value);

public:
    Communication();
    ~Communication();
    unsigned char* sendingPacket(uint16_t time);
    void encodePacket(uint16_t time);

    wingData *_wingData;
    packetData *_packetSerial;
};

#endif