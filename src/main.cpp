#include <Arduino.h>
#include <iostream>
#include <memory>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "INA219.hpp"
#include "Communication.hpp"


OrnibiBot robot;
Communication comm;

SBUS wing_left(&Serial1, true);
SBUS wing_right(&Serial2, true);

IntervalTimer interpolation;
IntervalTimer sbus;
IntervalTimer sensor;
IntervalTimer communication;

static uint16_t mid_left = 1500;
static uint16_t mid_right = 1500;

uint32_t time_start;
bool flag_start=0;

void commHandler(){

    comm._raw_data->timestamp       = (uint32_t)millis()-time_start;
    comm._raw_data->desiredLeft     = robot._wingPosition->desired_left;
    comm._raw_data->desiredRight    = robot._wingPosition->desired_right;
    comm._raw_data->positionLeft    = robot._wingPosition->actual_left;
    comm._raw_data->positionRight   = robot._wingPosition->actual_right;
    comm._raw_data->currentLeft     = 3.2f;
    comm._raw_data->currentRight    = 2.4f;
    comm._raw_data->voltageLeft     = 8.5f;
    comm._raw_data->voltageRight    = 7.8f;

    comm.sendingPacket(comm._raw_data);
}

void interpolationHandler(){
  robot._flappingParam->amplitude = 60;
  robot._flappingParam->frequency = 0.5;
  robot._flappingParam->offset = 0;

  int8_t signal = robot.flappingPattern(sine);

  robot._wingPosition->desired_left = wing_left.degToSignal(signal);
  robot._wingPosition->desired_right = wing_right.degToSignal(-signal);

  if(robot._flappingParam->time < robot.getFlapMs())  robot._flappingParam->time++;
  else  robot._flappingParam->time = 0;
}

void sbusHandler(){

    wing_left.setPosition(robot._wingPosition->desired_left);
    wing_right.setPosition(robot._wingPosition->desired_right);

}

void sensorHandler(){
    
}



void setup() {
  // Configure serial transport
  Serial.begin(460800);
  while(!Serial);

  interpolation.begin(interpolationHandler, 1000);
  sbus.begin(sbusHandler, 5000);
  sensor.begin(sensorHandler, 5000);
  communication.begin(commHandler, 5000);
  time_start = millis();
//   flag_start = 1;

}

void loop() {

}