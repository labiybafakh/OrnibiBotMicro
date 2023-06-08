#include <Arduino.h>
#include <iostream>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "INA219.hpp"
#include "Communication.hpp"


OrnibiBot robot;
Communication comm;

// SBUS wing_left(&Serial1, true);
// SBUS wing_right(&Serial2, true);

IntervalTimer interpolation;
IntervalTimer sbus;
IntervalTimer sensor;
IntervalTimer communication;

static uint16_t mid_left = 1500;
static uint16_t mid_right = 1500;

uint16_t time_now, time_last;

void commHandler(){
    time_now = uint16_t(millis()/1000) - time_last;
    comm.sendingPacket(time_now);
    time_last = time_now;
}

void interpolationHandler(){
  robot._flappingParam->amplitude = 10;
  robot._flappingParam->frequency = 3;
  robot._flappingParam->offset = 0;

  int8_t signal = robot.flappingPattern(sine);

  robot._wingPosition->desired_left = mid_left + signal;
  robot._wingPosition->desired_right = mid_right + signal;

  if(robot._flappingParam->time < robot.getFlapMs())  robot._flappingParam->time++;
  else  robot._flappingParam->time = 0;
}

void sbusHandler(){

    // wing_left.setPosition(robot._wingPosition->desired_left);
    // wing_right.setPosition(robot._wingPosition->desired_right);

}

void sensorHandler(){

}



void setup() {
  // Configure serial transport
  Serial.begin(460800);
  while(!Serial);

  interpolation.begin(interpolationHandler, 1000);
  sbus.begin(sbusHandler, 10000);
  sensor.begin(sensorHandler, 1000);
  communication.begin(commHandler, 500);

}

void loop() {
}