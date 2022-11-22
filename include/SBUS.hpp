#ifndef SBUS_HPP
#define SBUS_HPP

#include <array>


class SBUS{
    protected:
        int sbus_speed = 100000;    
        int sbus_servo_id[16];
        char sbus_data[25] = {
            0x0f, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00
        };
        int servo_position[5];
        char rx_sbus_data[25];
        char* _rx_sbus_data;
        
    public:
        SBUS();
        ~SBUS();
        void init();
        int degToSignal(int pos);
        void setPosition(int pos[]);
        bool sendPosition();
        char* get_sbus();
        void getPosition();

};

#endif