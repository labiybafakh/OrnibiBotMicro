#include <Arduino.h>
#include <iostream>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "INA219.hpp"
#include "Watchdog_t4.h"

WDT_T4<EWM> ewm;

void callbackWDT(){
    digitalToggle(13);
    Serial.println("Watchdog");
    ewm.reset();
}

struct packetData
{
    uint16_t timestamp; //2 bytes
    int16_t positionLeft; //2 bytes
    int16_t positionRight; //2 bytes
    int16_t currentLeft; //2 bytes
    int16_t currentRight; //2 bytes
    int16_t voltageLeft; //2 bytes
    int16_t voltageRight; //2 bytes
};

struct actuatorData
{
    float position;
    float current;
    float voltage;
};

int16_t encodeFloatToInt(float value){
    return static_cast<int16_t>(value * 100.0f);
}

float decodeFloatToInt(int16_t value){
    return value/100.0f;
}

actuatorData *_wingLeft;
actuatorData *_wingRight;

packetData *_packetData;

void sendingPacket(){
    byte _packet[14];

    _packet[0] = 0xFF;
    _packet[1] = _packetData->timestamp & 0xFF;
    _packet[2] = (_packetData->timestamp >> 8) & 0xFF;
    _packet[3] = _packetData->positionLeft & 0xFF;
    _packet[4] = (_packetData->positionLeft >> 8) & 0xFF;
    _packet[5] = _packetData->positionRight & 0xFF;
    _packet[6] = (_packetData->positionRight >> 8) & 0xFF;
    _packet[7] = _packetData->currentLeft & 0xFF;
    _packet[8] = (_packetData->currentLeft >> 8) & 0xFF;
    _packet[9] = _packetData->currentRight & 0xFF;
    _packet[10] = (_packetData->currentRight >> 8) & 0xFF;
    _packet[12] = _packetData->voltageLeft & 0xFF;
    _packet[12] = (_packetData->voltageLeft >> 8) & 0xFF;   
    _packet[13] = _packetData->voltageRight & 0xFF;
    _packet[14] = (_packetData->voltageRight >> 8) & 0xFF;  
    _packet[15] = 0x30;   

    //encode the packet into a packet array to send using serial.write
    Serial.write(_packet, 14);
    Serial.println("DONE");
}

void setup() {
    Serial.begin(115200);
    while (!Serial)
    Serial.println("TESTING");

    WDT_timings_t configewm;
    configewm.callback = callbackWDT;
    configewm.window = 100;
    configewm.timeout = 2000;
    configewm.pin   = 21;
    ewm.begin(configewm);
}

void loop() {
    _wingLeft->position = 2.33f;
    _wingLeft->current  = 1.32f;
    _wingLeft->voltage  = 8.4f;

    _wingRight->position = 1.33f;
    _wingRight->current  = 2.32f;
    _wingRight->voltage  = 7.4f;

    _packetData->timestamp      = millis();
    _packetData->positionLeft   = encodeFloatToInt(_wingLeft->position);
    _packetData->positionRight  = encodeFloatToInt(_wingRight->position);
    _packetData->currentLeft    = encodeFloatToInt(_wingRight->current);
    _packetData->currentRight   = encodeFloatToInt(_wingRight->current);
    _packetData->voltageLeft    = encodeFloatToInt(_wingLeft->voltage);
    _packetData->voltageRight   = encodeFloatToInt(_wingRight->voltage); 

    sendingPacket();
    delay(100);
}

