#include <Arduino.h>
#include <iostream>
#include <memory>
#include <unistd.h>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "Communication.hpp"
#include "DFRobot_INA219.h"
#include <string.h>

typedef struct{
  volatile uint32_t start_time;
  volatile uint32_t last_time;
  uint32_t total_time;
  volatile uint32_t counter;
  volatile uint8_t flag;
} wing_raw_data;

OrnibiBot robot;
Communication comm;

volatile bool test;

SBUS wing_left(&Serial1, true);
SBUS wing_right(&Serial2, true);


DFRobot_INA219_IIC power_left(&Wire1, INA219_I2C_ADDRESS1);
DFRobot_INA219_IIC power_right(&Wire1, INA219_I2C_ADDRESS3);
DFRobot_INA219_IIC power_source(&Wire2, INA219_I2C_ADDRESS1);


IntervalTimer interpolation;
IntervalTimer sbus;
IntervalTimer sensor;
IntervalTimer communication;

wing_raw_data *p_wing_left_raw;
wing_raw_data *p_wing_right_raw;

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

    // Serial.println(robot._wingPower->current_left);
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

  p_wing_left_raw->counter++;
  p_wing_right_raw->counter++;

  if(digitalReadFast(23) == LOW) {
    if(p_wing_left_raw->flag == 0){
      p_wing_left_raw->total_time = p_wing_left_raw->counter;
      p_wing_left_raw->flag = 1;
      p_wing_left_raw->counter = 0;
    }
    else p_wing_left_raw->counter = 0;
  }
  else if(digitalReadFast(23) == HIGH){
   p_wing_left_raw->flag = 0;
  }

  if(digitalReadFast(22) == LOW) {
    if(p_wing_right_raw->flag == 0){
      p_wing_right_raw->total_time = p_wing_right_raw->counter;
      p_wing_right_raw->flag = 1;
      p_wing_right_raw->counter = 0;
    }
    else p_wing_right_raw->counter = 0;
  }
  else if(digitalReadFast(22) == HIGH){
   p_wing_right_raw->flag = 0;
  }

}

void setup() {
  // Configure serial transport
  Serial.begin(460800);
  // Serial.begin(115200);

  pinMode(22, INPUT);
  pinMode(23, INPUT);
  pinMode(21, OUTPUT);

  power_left.begin();
  power_right.begin();
  power_source.begin();
  
  // setupGPT1();
  
  p_wing_left_raw = (wing_raw_data *)malloc(sizeof(wing_raw_data));
  p_wing_right_raw = (wing_raw_data *)malloc(sizeof(wing_raw_data));

  p_wing_left_raw->flag = 0;
  p_wing_right_raw->flag = 0;
  
  delay(100);
  power_left.linearCalibrate(ina219Reading_mA, extMeterReading_mA); delay(100);
  power_right.linearCalibrate(ina219Reading_mA, extMeterReading_mA);  delay(100);
  power_source.linearCalibrate(ina219Reading_mA, extMeterReading_mA); delay(100);

  while(!Serial);
  sensor.begin(sensorHandler, 1);
  sensor.priority(0);
  interpolation.begin(interpolationHandler, 1000);
  interpolation.priority(1);
  sbus.begin(sbusHandler, 10000);
  sbus.priority(2);
  communication.begin(commHandler, 5000);
  communication.priority(3);
  time_start = millis();
//   flag_start = 1;

}

void loop() {

  if(millis()%5==0){
    robot._wingPower->current_left = power_left.getCurrent_mA();
    robot._wingPower->current_right = power_right.getCurrent_mA();
    robot._wingPower->voltage_left  = power_left.getBusVoltage_V();
    robot._wingPower->voltage_right = power_right.getBusVoltage_V();
  }

  // String data = (String)robot._wingPower->current_left + "\t" + (String)robot._wingPower->current_right;
  // Serial.println(data);
  // delay(5);
}
