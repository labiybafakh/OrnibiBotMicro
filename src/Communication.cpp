#include "Communication.hpp"

Communication::Communication()
{
    // _serial_dev = serial_com;
}

Communication::~Communication()
{
}

int16_t Communication::encodeFloatToInt(float value)
{
    return static_cast<int16_t>(value * 100.0f);
}

float Communication::decodeFloatToInt(int16_t value)
{
    return value / 100.0f;
}

unsigned char* Communication::sendingPacket(usb_serial_class* _serial){
    unsigned char _packet[16];
    
    Communication::encodePacket();

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
    _serial->write(_packet, 16);

    return _packet;
}

void Communication::encodePacket(){
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
}