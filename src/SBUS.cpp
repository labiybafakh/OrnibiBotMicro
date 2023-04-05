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

int SBUS::degToSignal(int pos){
    //Rotate Servo from -60 to 60 Degrees
    //Mid Servo using SBUS is 1023
    //Upstroke<1023 - Downstroke>1023
    if(pos>60)         pos=60;
    else if(pos<-60)   pos=-60;

    return (int)(1023 - (double)(-pos*17)); //reversed to adjust upstroke-downstroke
}
void SBUS::setPosition(int pos[]){
    //Convert Position in degree to signal
    sbus_servo_id[0] = SBUS::degToSignal(pos[0]);
    sbus_servo_id[1] = SBUS::degToSignal(pos[1]);
    sbus_servo_id[2] = SBUS::degToSignal(pos[2]);
    sbus_servo_id[3] = SBUS::degToSignal(pos[3]);
    sbus_servo_id[4] = 0;
    sbus_servo_id[5] = 0;

    //Encode Servo Position into a packet data
    sbus_data[0] = 0x0f;
    sbus_data[1] =  sbus_servo_id[0] & 0xff;
    sbus_data[2] = ((sbus_servo_id[0] >> 8) & 0x07 ) | ((sbus_servo_id[1] << 3 ) );
    sbus_data[3] = ((sbus_servo_id[1] >> 5) & 0x3f ) | (sbus_servo_id[2]  << 6);
    sbus_data[4] = ((sbus_servo_id[2] >> 2) & 0xff ) ;
    sbus_data[5] = ((sbus_servo_id[2] >> 10) & 0x01 ) | (sbus_servo_id[3] << 1 )   ;
    sbus_data[6] = ((sbus_servo_id[3] >> 7) & 0x0f ) | (sbus_servo_id[4]  << 4 )   ;
    sbus_data[7] = ((sbus_servo_id[4] >> 4) & 0x7f ) | (sbus_servo_id[5]  << 7 )   ;
    sbus_data[8] = ((sbus_servo_id[5] >> 1) & 0xff ) ;
    sbus_data[9] = ((sbus_servo_id[5] >> 9) & 0x03 ) ;

}

bool SBUS::sendPosition(){
    //Sending packet data to SBUS
    return serial_dev_->write(sbus_data, 25);
}


char *SBUS::get_sbus(){
    char header;
    char position[4];

    return _rx_sbus_data;
}

void SBUS::getPosition(){
    
}