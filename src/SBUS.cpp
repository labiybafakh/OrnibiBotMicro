#include "SBUS.hpp"
#include <Arduino.h>


SBUS::SBUS(HardwareSerial *serial_dev, bool invert){
    serial_dev_ = serial_dev;
    _rx_sbus_data = rx_sbus_data;
    
    if(invert)
        ((HardwareSerial *)serial_dev_)->begin(sbus_speed, SERIAL_8E2_TXINV);
    else
        ((HardwareSerial *)serial_dev_)->begin(sbus_speed, SERIAL_8E2);
}

SBUS::~SBUS(){
    ((HardwareSerial *)serial_dev_)->end();
}

uint16_t SBUS::degToSignal(int8_t pos){
    //Rotate Servo from -60 to 60 Degrees
    //Mid Servo using SBUS is 1023
    //Upstroke<1023 - Downstroke>1023
    if(pos>60)         pos=60;
    else if(pos<-60)   pos=-60;

    return (uint16_t)(1520 - (-pos*10)); //reversed to adjust upstroke-downstroke
}
// bool SBUS::setPosition(uint16_t pos[]){
//     //Convert Position in degree to signal
//     sbus_servo_id[0] = SBUS::degToSignal(pos[0]);
//     sbus_servo_id[1] = SBUS::degToSignal(pos[1]);
//     sbus_servo_id[2] = SBUS::degToSignal(pos[2]);
//     sbus_servo_id[3] = SBUS::degToSignal(pos[3]);
//     sbus_servo_id[4] = 0;
//     sbus_servo_id[5] = 0;

//     //Encode Servo Position into a packet data
//     sbus_data[0] = 0x0f;
//     sbus_data[1] =  sbus_servo_id[0] & 0xff;
//     sbus_data[2] = ((sbus_servo_id[0] >> 8) & 0x07 ) | ((sbus_servo_id[1] << 3 ) );
//     sbus_data[3] = ((sbus_servo_id[1] >> 5) & 0x3f ) | (sbus_servo_id[2]  << 6);
//     sbus_data[4] = ((sbus_servo_id[2] >> 2) & 0xff ) ;
//     sbus_data[5] = ((sbus_servo_id[2] >> 10) & 0x01 ) | (sbus_servo_id[3] << 1 )   ;
//     sbus_data[6] = ((sbus_servo_id[3] >> 7) & 0x0f ) | (sbus_servo_id[4]  << 4 )   ;
//     sbus_data[7] = ((sbus_servo_id[4] >> 4) & 0x7f ) | (sbus_servo_id[5]  << 7 )   ;
//     sbus_data[8] = ((sbus_servo_id[5] >> 1) & 0xff ) ;
//     sbus_data[9] = ((sbus_servo_id[5] >> 9) & 0x03 ) ;

//     return sendPosition(sbus_data);
// }

bool SBUS::setPosition(uint16_t pos){
    uint8_t packet_sbus[SBUS_BUFFER];
    memset(packet_sbus, 0x00, SBUS_BUFFER);

    // Encode Servo Position for channel 0 into a packet data
    packet_sbus[0] = 0x0F;  // Start byte
    packet_sbus[1] = pos & 0xFF;  // Lower 8 bits of channel 0
    packet_sbus[2] = (pos >> 8) & 0x07;  // Upper 3 bits of channel 0
    // Stop byte(s) can be set according to the specifics of your receiver
    packet_sbus[23] = 0x00;
    packet_sbus[24] = 0x00;

    if(pos < 1023 || pos > 2047) {
        return 0;
    }
    else
        return sendPosition(packet_sbus);
}

bool SBUS::sendPosition(uint8_t data[SBUS_BUFFER]){
    //Sending packet data to SBUS
    return serial_dev_->write(data, SBUS_BUFFER);
}