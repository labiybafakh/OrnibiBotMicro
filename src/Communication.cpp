#include "Communication.hpp"
#include <stddef.h>

Communication::Communication()
{
    // Serial.begin(115200);
    _packetSerial   = (packetData *)malloc(sizeof(packetData));
    _wingData       = (wingData *)malloc(sizeof(wingData));
    _raw_data       = (packetData *)malloc(sizeof(packetData));
}

Communication::~Communication()
{
    free(_packetSerial);
    free(_wingData);
    free(_raw_data);

}

int16_t Communication::encodeFloatToInt(float value)
{
    return static_cast<int16_t>(value * 100.0f);
}

float Communication::decodeFloatToInt(int16_t value)
{
    return value / 100.0f;
}

unsigned char* Communication::sendingPacket(packetData* raw){
    uint8_t _packet[buffer_size];
    
    Communication::encodePacket(raw);

    _packet[0] = _packetSerial->timestamp & 0xFF;
    _packet[1] = (_packetSerial->timestamp >> 8) & 0xFF;
    _packet[2] = (_packetSerial->timestamp >> 16) & 0xFF;
    _packet[3] = (_packetSerial->timestamp >> 24) & 0xFF;
    _packet[4] = _packetSerial->desiredLeft & 0xFF;
    _packet[5] = (_packetSerial->desiredLeft >> 8) & 0xFF;
    _packet[6] = _packetSerial->desiredRight & 0xFF;
    _packet[7] = (_packetSerial->desiredRight >> 8) & 0xFF;
    _packet[8] = _packetSerial->positionLeft & 0xFF;
    _packet[9] = (_packetSerial->positionLeft >> 8) & 0xFF;
    _packet[10] = _packetSerial->positionRight & 0xFF;
    _packet[11] = (_packetSerial->positionRight >> 8) & 0xFF;
    _packet[12] = _packetSerial->currentLeft & 0xFF;
    _packet[13] = (_packetSerial->currentLeft >> 8) & 0xFF;
    _packet[14] = _packetSerial->currentRight & 0xFF;
    _packet[15] = (_packetSerial->currentRight >> 8) & 0xFF;
    _packet[16] = _packetSerial->voltageLeft & 0xFF;
    _packet[17] = (_packetSerial->voltageLeft >> 8) & 0xFF;   
    _packet[18] = _packetSerial->voltageRight & 0xFF;
    _packet[19] = (_packetSerial->voltageRight >> 8) & 0xFF;  


    //encode the packet into a packet array to send using serial.write
    Serial.write(_packet, buffer_size);

    return _packet;
}

void Communication::encodePacket(packetData* raw)
{
    _packetSerial->timestamp      = raw->timestamp;
    _packetSerial->desiredLeft    = raw->desiredLeft;
    _packetSerial->desiredRight   = raw->desiredRight;
    _packetSerial->positionLeft   = encodeFloatToInt(raw->positionLeft);
    _packetSerial->positionRight  = encodeFloatToInt(raw->positionRight);
    _packetSerial->currentLeft    = encodeFloatToInt(raw->currentLeft);
    _packetSerial->currentRight   = encodeFloatToInt(raw->currentRight);
    _packetSerial->voltageLeft    = encodeFloatToInt(raw->voltageLeft);
    _packetSerial->voltageRight   = encodeFloatToInt(raw->voltageRight); 

}