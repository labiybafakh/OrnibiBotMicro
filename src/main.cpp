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

Communication _comm;
OrnibiBot _ornibibot;

void callbackWDT(){
    digitalToggle(13);
    Serial.println("Watchdog");
    // ewm.reset();
}

void setup() {
    Serial.begin(480000);
    while (!Serial)
    Serial.println("TESTING");

    // WDT_timings_t configewm;
    // configewm.callback = callbackWDT;
    // configewm.window = 100;
    // configewm.timeout = 2000;
    // configewm.pin   = 21;
    // ewm.begin(configewm);
}

void loop() {
    _comm.sendingPacket();
    _ornibibot.flaps(1.1f,1);
}

