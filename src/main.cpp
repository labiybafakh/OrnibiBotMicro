#ifdef DEBUG //Remove compiler optimizations for hardware debugging
#pragma GCC optimize ("O0") 
#endif

#include <Arduino.h>
#include <iostream>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "Communication.hpp"
#include "INA219.hpp"
#include "Watchdog_t4.h"

// WDT_T4<EWM> ewm;

SBUS wing_left(&Serial1, true);
SBUS wing_right(&Serial2, true);


Communication _comm;
OrnibiBot robot;

int targetServo[4];


void callbackWDT(){
    digitalToggle(13);
    Serial.println("Watchdog");
    // ewm.reset();
}

void setup() {
    Serial.begin(460800);
    SerialUSB1.begin(115200);
    // while (!Serial);
    // Serial.println("TESTING");

    // WDT_timings_t configewm;
    // configewm.callback = callbackWDT;
    // configewm.window = 100;
    // configewm.timeout = 2000;
    // configewm.pin   = 21;
    // ewm.begin(configewm);
}

void loop() {
    robot._flapFreq = 5;
    robot._amplitude = 30;
    robot._offset = 0;

    // flapping.setPosition(targetServo);
    // flapping.sendPosition();
    targetServo[0] = robot.flappingPattern(sine);
    targetServo[1] = robot.flappingPattern(sine);

    wing_left.setPosition(targetServo);
    wing_left.sendPosition();

    wing_right.setPosition(targetServo);
    wing_right.sendPosition();

    if (robot._time < robot._periode)
        robot._time++;
    else
        robot._time = 0;

    _comm.sendingPacket(&Serial);

    delayMicroseconds(500);
    // _ornibibot.flaps(1.1f,1);
}

