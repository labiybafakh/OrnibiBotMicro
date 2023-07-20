#include "Communication.hpp"
#include <stddef.h>

Communication::Communication()
{
    // Serial.begin(115200);
    _packetSerial   = (packetData *)malloc(sizeof(packetData));
    _wingData       = (wingData *)malloc(sizeof(wingData));
    _raw_data       = (rawData *)malloc(sizeof(rawData));
}

Communication::~Communication()
{
    free(_packetSerial);
    free(_wingData);
    free(_raw_data);

}

int16_t Communication::encodeFloatToInt(float value)
{
    return (int16_t)(value * 100);
}

float Communication::decodeFloatToInt(int16_t value)
{
    return value / 100.0f;
}

unsigned char* Communication::sendingPacket(rawData* raw){
    uint8_t _packet[buffer_size];
    
    Communication::encodePacket(raw);

    _packet[0] = _packetSerial->timestamp & 0xFF;
    _packet[1] = (_packetSerial->timestamp >> 8) & 0xFF;
    _packet[2] = (_packetSerial->timestamp >> 16) & 0xFF;
    _packet[3] = (_packetSerial->timestamp >> 24) & 0xFF;
    _packet[4] = _packetSerial->desired_left & 0xFF;
    _packet[5] = (_packetSerial->desired_left >> 8) & 0xFF;
    _packet[6] = _packetSerial->desired_right & 0xFF;
    _packet[7] = (_packetSerial->desired_right >> 8) & 0xFF;
    _packet[8] = _packetSerial->actual_left & 0xFF;
    _packet[9] = (_packetSerial->actual_left >> 8) & 0xFF;
    _packet[10] = _packetSerial->actual_right & 0xFF;
    _packet[11] = (_packetSerial->actual_right >> 8) & 0xFF;
    _packet[12] = _packetSerial->power_left & 0xFF;
    _packet[13] = (_packetSerial->power_left >> 8) & 0xFF;
    _packet[14] = _packetSerial->power_right & 0xFF;
    _packet[15] = (_packetSerial->power_right >> 8) & 0xFF;
    // _packet[16] = _packetSerial->voltageLeft & 0xFF;
    // _packet[17] = (_packetSerial->voltageLeft >> 8) & 0xFF;   
    // _packet[18] = _packetSerial->voltageRight & 0xFF;
    // _packet[19] = (_packetSerial->voltageRight >> 8) & 0xFF;  


    //encode the packet into a packet array to send using serial.write
    Serial.write(_packet, buffer_size);

    return _packet;
}

void Communication::encodePacket(rawData* raw)
{
    _packetSerial->timestamp      = raw->timestamp;
    _packetSerial->desired_left    = encodeFloatToInt(raw->desired_left);
    _packetSerial->desired_right   = encodeFloatToInt(raw->desired_right);
    _packetSerial->actual_left   = encodeFloatToInt(raw->actual_left);
    _packetSerial->actual_right  = encodeFloatToInt(raw->actual_right);
    _packetSerial->power_left    = encodeFloatToInt(raw->power_left);
    _packetSerial->power_right   = encodeFloatToInt(raw->power_right);
    // _packetSerial->voltageLeft    = encodeFloatToInt(raw->voltageLeft);
    // _packetSerial->voltageRight   = encodeFloatToInt(raw->voltageRight); 

}