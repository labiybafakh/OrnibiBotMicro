#ifndef ORNIBIBOT_HPP
#define ORNIBIBOT_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <iostream>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string.h>
// #include "OrnibiBot.hpp"


class OrnibiBot{
        
    private:
        volatile uint16_t getFlapMs();
        struct tailPosition{
            unsigned int roll;
            unsigned int pitch;
        };

    public:
        volatile int16_t _offset;
        volatile uint16_t _time;
        volatile uint16_t _amplitude;
        volatile double _flapFreq;
        volatile uint16_t _periode;
        volatile uint16_t _flapping;

        volatile int16_t sineFlap();
        volatile int16_t squareFlap();
        volatile int16_t sawFlap();
        volatile int16_t triangleFlap();
        volatile int16_t reverse_sawFlap();

        tailPosition tail_position;

};

#endif