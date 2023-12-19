#ifndef SBUS_HPP
#define SBUS_HPP

#include <array>
#include <iostream>
#include "HardwareSerial.h"
#define SBUS_BUFFER 25
#define SBUS_CHANNEL 16


class SBUS{
    protected:

        int sbus_speed = 100000;    
        int sbus_servo_id[SBUS_CHANNEL];
        uint16_t sbus_data[SBUS_BUFFER] = {
            0x0f, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00
        };
        int servo_position[5];
        char rx_sbus_data[SBUS_BUFFER];
        char* _rx_sbus_data;
        Stream *serial_dev_;
        bool invert_flag_;
        
    public:
        SBUS(HardwareSerial *serial_dev, bool invert);
        ~SBUS();
        void init();
        uint16_t degToSignal(int8_t pos);
        // bool setPosition(uint16_t pos[]);
        bool setPosition(uint16_t pos);
        bool sendPosition(uint8_t data[SBUS_BUFFER]);
        void begin();
        // char* get_sbus();
        // void getPosition();

};

#endif