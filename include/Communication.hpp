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
        int16_t desired_left;   // 2 bytes
        int16_t desired_right;  // 2 bytes
        int16_t actual_left;  // 2 bytes
        int16_t actual_right; // 2 bytes
        int16_t power_left;   // 2 bytes
        int16_t power_right;  // 2 bytes
    } packetData;

    typedef struct
    {
        uint32_t timestamp;    // 4 bytes
        float desired_left;   // 2 bytes
        float desired_right;  // 2 bytes
        float actual_left;  // 2 bytes
        float actual_right; // 2 bytes
        float power_left;   // 2 bytes
        float power_right;  // 2 bytes
    } rawData;

    struct actuatorData
    {
        float position;
        float velocity;
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