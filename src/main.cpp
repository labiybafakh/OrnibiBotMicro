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

actuatorData wingLeft;
actuatorData wingRight;
actuatorData *_wingLeft = &wingLeft;
actuatorData *_wingRight = &wingRight;

packetData packetSerial;
packetData *_packetSerial = &packetSerial;

void sendingPacket(){
    byte _packet[16];

    _packet[0] = 0xFF;
    _packet[1] = _packetSerial->timestamp & 0xFF;
    _packet[2] = (_packetSerial->timestamp >> 8) & 0xFF;
    _packet[3] = _packetSerial->positionLeft & 0xFF;
    _packet[4] = (_packetSerial->positionLeft >> 8) & 0xFF;
    _packet[5] = _packetSerial->positionRight & 0xFF;
    _packet[6] = (_packetSerial->positionRight >> 8) & 0xFF;
    _packet[7] = _packetSerial->currentLeft & 0xFF;
    _packet[8] = (_packetSerial->currentLeft >> 8) & 0xFF;
    _packet[9] = _packetSerial->currentRight & 0xFF;
    _packet[10] = (_packetSerial->currentRight >> 8) & 0xFF;
    _packet[12] = _packetSerial->voltageLeft & 0xFF;
    _packet[12] = (_packetSerial->voltageLeft >> 8) & 0xFF;   
    _packet[13] = _packetSerial->voltageRight & 0xFF;
    _packet[14] = (_packetSerial->voltageRight >> 8) & 0xFF;  
    _packet[15] = 0x30;   

    //encode the packet into a packet array to send using serial.write
    Serial.write(_packet, 16);
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

    _packetSerial->timestamp      = millis();
    _packetSerial->positionLeft   = encodeFloatToInt(_wingLeft->position);
    _packetSerial->positionRight  = encodeFloatToInt(_wingRight->position);
    _packetSerial->currentLeft    = encodeFloatToInt(_wingRight->current);
    _packetSerial->currentRight   = encodeFloatToInt(_wingRight->current);
    _packetSerial->voltageLeft    = encodeFloatToInt(_wingLeft->voltage);
    _packetSerial->voltageRight   = encodeFloatToInt(_wingRight->voltage); 

    sendingPacket();
    delay(1);
}

