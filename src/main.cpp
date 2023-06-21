#include <Arduino.h>
#include <iostream>
#include <memory>
#include <unistd.h>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
// #include "INA219.hpp"
#include "Communication.hpp"
#include "DFRobot_INA219.h"


OrnibiBot robot;
Communication comm;

SBUS wing_left(&Serial1, true);
SBUS wing_right(&Serial2, true);


DFRobot_INA219_IIC power_left(&Wire1, 0x40);
DFRobot_INA219_IIC power_right(&Wire1, 0x44);
DFRobot_INA219_IIC power_source(&Wire2, 0x40);

// INA219 power_left(0x40, &Wire1);
// INA219 power_right(0x44, &Wire1);
// INA219 power_source(0x40, &Wire2);


IntervalTimer interpolation;
IntervalTimer sbus;
IntervalTimer sensor;
IntervalTimer communication;

static uint16_t mid_left = 1500;
static uint16_t mid_right = 1500;

uint32_t time_start;
bool flag_start=0;
unsigned char flag_run = 0;

float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;

void commHandler(){

    comm._raw_data->timestamp       = (uint32_t)millis()-time_start;
    comm._raw_data->desiredLeft     = robot._wingPosition->desired_left;
    comm._raw_data->desiredRight    = robot._wingPosition->desired_right;
    comm._raw_data->positionLeft    = robot._wingPosition->actual_left;
    comm._raw_data->positionRight   = robot._wingPosition->actual_right;
    comm._raw_data->currentLeft     = robot._wingPower->current_left;
    comm._raw_data->currentRight    = robot._wingPower->current_right;
    comm._raw_data->voltageLeft     = robot._wingPower->voltage_left;
    comm._raw_data->voltageRight    = robot._wingPower->voltage_right;

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
    robot._wingPower->current_left = power_left.getCurrent_mA();
    robot._wingPower->current_right = power_right.getCurrent_mA();
    robot._wingPower->voltage_left  = power_left.getBusVoltage_V();
    robot._wingPower->voltage_right = power_right.getBusVoltage_V();
}


void setup() {
  // Configure serial transport
  // Serial.begin(460800);
  Serial.begin(115200);

  power_left.begin();
  power_right.begin();
  power_source.begin();
  delay(100);
  power_left.linearCalibrate(ina219Reading_mA, extMeterReading_mA); delay(100);
  power_right.linearCalibrate(ina219Reading_mA, extMeterReading_mA);  delay(100);
  power_source.linearCalibrate(ina219Reading_mA, extMeterReading_mA); delay(100);

  while(!Serial);

  interpolation.begin(interpolationHandler, 1000);
  sbus.begin(sbusHandler, 5000);
  // sensor.begin(sensorHandler, 5000);
  communication.begin(commHandler, 5000);
  time_start = millis();
//   flag_start = 1;

}

void loop() {
  // Serial.println(power_right.getCurrent_mA());
  // delay(1000);

}

// void serialEvent(){
//   if(Serial.available()){
//     flag_run = Serial.read();
//   }
// }