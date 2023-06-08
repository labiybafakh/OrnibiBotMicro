#include "Communication.hpp"

Communication::Communication()
{
    // Serial.begin(115200);
    _packetSerial   = (packetData *)malloc(sizeof(packetData));
    _wingData       = (wingData *)malloc(sizeof(wingData));
}

Communication::~Communication()
{
    free(_packetSerial);
    free(_wingData);

}

int16_t Communication::encodeFloatToInt(float value)
{
    return static_cast<int16_t>(value * 100.0f);
}

float Communication::decodeFloatToInt(int16_t value)
{
    return value / 100.0f;
}

unsigned char* Communication::sendingPacket(uint16_t time){
    uint8_t _packet[16];
    
    Communication::encodePacket(time);

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

    return _packet;
}

void Communication::encodePacket(uint16_t time){

    _wingData->left.position = 2.33f;
    _wingData->left.current  = 1.32f;
    _wingData->left.voltage  = 8.4f;

    _wingData->right.position = 1.33f;
    _wingData->right.current  = 2.32f;
    _wingData->right.voltage  = 7.4f;

    _packetSerial->timestamp      = time;
    _packetSerial->positionLeft   = encodeFloatToInt(_wingData->left.position);
    _packetSerial->positionRight  = encodeFloatToInt(_wingData->right.position);
    _packetSerial->currentLeft    = encodeFloatToInt(_wingData->left.current);
    _packetSerial->currentRight   = encodeFloatToInt(_wingData->right.current);
    _packetSerial->voltageLeft    = encodeFloatToInt(_wingData->left.voltage);
    _packetSerial->voltageRight   = encodeFloatToInt(_wingData->right.voltage); 

}