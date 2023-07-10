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
        uint32_t timestamp;    // 4 bytes
        uint16_t desiredLeft;   // 2 bytes
        uint16_t desiredRight;  // 2 bytes
        int16_t positionLeft;  // 2 bytes
        int16_t positionRight; // 2 bytes
        int16_t powerleft;   // 2 bytes
        int16_t powerright;  // 2 bytes
    } packetData;

    typedef struct
    {
        uint32_t timestamp;    // 4 bytes
        uint16_t desiredLeft;   // 2 bytes
        uint16_t desiredRight;  // 2 bytes
        float positionLeft;  // 2 bytes
        float positionRight; // 2 bytes
        float powerleft;   // 2 bytes
        float powerright;  // 2 bytes
    } rawData;

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

    static const uint8_t buffer_size = 16;

public:
    Communication();
    ~Communication();
    unsigned char* sendingPacket(rawData* raw);
    void encodePacket(rawData* raw);

    wingData *_wingData;
    packetData *_packetSerial;
    // packetData *_raw_data;
    rawData *_raw_data;
};

#endif