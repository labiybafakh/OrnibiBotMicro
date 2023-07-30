#include <Arduino.h>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <math.h>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "Communication.hpp"
#include "DFRobot_INA219.h"
#include <string.h>
#define LEFT 0
#define RIGHT 1

uint32_t previous_time;
uint32_t current_time;

OrnibiBot robot;
Communication comm;

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

// static uint16_t mid_left = 1500;
// static uint16_t mid_right = 1500;

uint32_t time_start;
bool flag_start=0;
unsigned char flag_run = 0;

float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;

int8_t WingPositionDeg(uint8_t wing_, volatile uint16_t& pulse_data){
  if(wing_ == left ) return (int8_t) ((702 - pulse_data) / 2.557f); 
  else return (int8_t) -1 * ((637 - pulse_data) / 2.428f) ; 
}

volatile float WingPositionRads(uint8_t wing_, volatile uint16_t& pulse_data){
  return (float) WingPositionDeg(wing_, pulse_data) * 0.0174533f;
}

float DegToRads(volatile int8_t degree){
  return (float) degree * (M_PI/180.0f);
}



void commHandler(){

    comm._raw_data->timestamp             = (uint32_t)millis()-time_start;
    comm._raw_data->desired_left          = robot.p_wing_data->desired_left;
    comm._raw_data->desired_right         = robot.p_wing_data->desired_right;
    comm._raw_data->actual_left           = robot.p_wing_data->actual_left;
    comm._raw_data->actual_right          = robot.p_wing_data->actual_right;
    comm._raw_data->power_left            = robot.p_wing_data->power_left;
    comm._raw_data->power_right           = robot.p_wing_data->power_right;

    comm.sendingPacket(comm._raw_data);

    // String data = (String)robot.p_wing_data->power_left + "\t" + (String)robot.p_wing_data->power_right;
    // Serial.println(data);
}

void interpolationHandler(){
  robot._flappingParam->amplitude = 30;
  robot._flappingParam->frequency = 3.5;
  robot._flappingParam->offset = 20;

  // robot._flappingParam->signal = 0;
  robot._flappingParam->signal = robot.flappingPattern(square);


  robot.p_wing_data->desired_left = DegToRads(robot._flappingParam->signal);
  robot.p_wing_data->desired_right = DegToRads(robot._flappingParam->signal);

  if(robot._flappingParam->time < robot.getFlapMs())  robot._flappingParam->time++;
  else  robot._flappingParam->time = 0;
}

void sbusHandler(){

    wing_left.setPosition(wing_left.degToSignal(robot._flappingParam->signal));
    wing_right.setPosition(wing_right.degToSignal(-robot._flappingParam->signal));
    //     wing_left.setPosition(wing_left.degToSignal(0));
    // wing_right.setPosition(wing_right.degToSignal(0));
    // wing_left.setPosition(wing);
    // wing_right.setPosition(iter);

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
  sbus.begin(sbusHandler, 5000);
  sbus.priority(2);
  communication.begin(commHandler, 5000);
  communication.priority(3);
  time_start = millis();
//   flag_start = 1;

}

void loop() {

  current_time = millis();
  
  if((current_time - previous_time) > 5 ){
    robot.p_wing_data->power_left = (float) power_left.getPower_mW() * 0.62f * 0.001f;
    robot.p_wing_data->power_right = (float) power_right.getPower_mW() * 0.62f * 0.001f;
    robot.p_wing_data->actual_left = WingPositionRads(left, p_wing_left_raw->total_time);
    robot.p_wing_data->actual_right = WingPositionRads(right, p_wing_right_raw->total_time);

    previous_time = current_time;
  }

}

// void serialEvent(){
//   if(Serial.available())
// }