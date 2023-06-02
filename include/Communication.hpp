#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include <iostream>
#include <stdio.h>
#include <Arduino.h>
#include "HardwareSerial.h"

class Communication
{
private:
    struct packetData
    {
        uint16_t timestamp;    // 2 bytes
        int16_t positionLeft;  // 2 bytes
        int16_t positionRight; // 2 bytes
        int16_t currentLeft;   // 2 bytes
        int16_t currentRight;  // 2 bytes
        int16_t voltageLeft;   // 2 bytes
        int16_t voltageRight;  // 2 bytes
    };

    struct actuatorData
    {
        float position;
        float current;
        float voltage;
    };

    actuatorData wingLeft;
    actuatorData wingRight;
    actuatorData *_wingLeft = &wingLeft;
    actuatorData *_wingRight = &wingRight;

    packetData packetSerial;
    packetData *_packetSerial = &packetSerial;

    int16_t encodeFloatToInt(float value);

    float decodeFloatToInt(int16_t value);
    Stream *_serial_com;

public:
    Communication();
    ~Communication();
    unsigned char* sendingPacket(usb_serial_class* _serial);
    void encodePacket();
};

#endif