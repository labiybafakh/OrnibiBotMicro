#include <Arduino.h>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <math.h>
#include "SBUS.hpp"
#include "OrnibibBot.hpp"
#include "Communication.hpp"
#include "DFRobot_INA219.h"
#include "ams.h"
#include <string.h>
#define LEFT 0
#define RIGHT 1

#define ADDRESS_1 (0x41)
#define ADDRESS_2 (0x43)

enum encoder_mode{
  degree=3,
  radian=4
};

bool debugging = 0;

uint32_t previous_time;
uint32_t current_time;

OrnibiBot robot;
Communication comm;

SBUS wing_left(&Serial1, true);
SBUS wing_right(&Serial2, true);

AMS_AS5048B encoder_left(ADDRESS_1);
AMS_AS5048B encoder_right(ADDRESS_2);

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
float mid_left = 267.7;
float mid_right = 261.7; 
bool flag_stop_frequency = 0;

uint16_t timer_frequency=0;

const size_t n_data = 7;
uint8_t serial_data[n_data];

int8_t WingPositionDeg(uint8_t wing_, uint16_t& pulse_data){
  if(wing_ == left ) return (int8_t) ((702 - pulse_data) / 2.557f); 
  else return (int8_t) -1 * ((637 - pulse_data) / 2.428f) ; 
}

volatile float WingPositionRads(uint8_t wing_, uint16_t& pulse_data){
  return (float) WingPositionDeg(wing_, pulse_data) * 0.0174533f;
}

float DegToRads(float degree){
  return (float) degree * (M_PI/180.0f);
}

void commHandler(){

  if(debugging){
    String data = (String)robot._flappingParam->frequency + "," + (String)robot._flappingParam->offset + "," + (String)robot._flappingParam->pattern;
    Serial.println(robot._flappingParam->signal);
  }
    
  else{
    comm._raw_data->timestamp             = (uint32_t)millis()-time_start;
    comm._raw_data->desired_left          = robot.p_wing_data->desired_left;
    comm._raw_data->desired_right         = robot.p_wing_data->desired_right;
    comm._raw_data->actual_left           = robot.p_wing_data->actual_left;
    comm._raw_data->actual_right          = robot.p_wing_data->actual_right;
    comm._raw_data->power_left            = robot.p_wing_data->power_left;
    comm._raw_data->power_right           = robot.p_wing_data->power_right;

    comm.sendingPacket(comm._raw_data);
  }

}

void interpolationHandler(){

  robot._flappingParam->rolling = 0;

  if(robot._flappingParam->frequency > 0.01f){
    robot._flappingParam->signal = robot.flappingPattern(robot._flappingParam->pattern, robot._flappingParam->down_stroke_percentage);
    robot.p_wing_data->desired_left = DegToRads(robot._flappingParam->signal - robot._flappingParam->rolling);
    robot.p_wing_data->desired_right = DegToRads(robot._flappingParam->signal + robot._flappingParam->rolling);
  }
  else{
    robot._flappingParam->signal = 0;
    robot.p_wing_data->desired_left = DegToRads(0);
    robot.p_wing_data->desired_right = DegToRads(0);
  }

  if(robot._flappingParam->time < robot.getFlapMs())  robot._flappingParam->time++;
  else  robot._flappingParam->time = 0;

}

void sbusHandler(){

  uint16_t left_position = wing_left.degToSignal(robot._flappingParam->signal);
  uint16_t right_position = wing_right.degToSignal(-robot._flappingParam->signal);

  wing_left.setPosition(left_position);
  wing_right.setPosition(right_position);

}

void sensorHandler(){
    robot.p_wing_data->actual_left = DegToRads(encoder_left.angleR(degree, true) - mid_left) * -1;
    robot.p_wing_data->actual_right = DegToRads(encoder_right.angleR(degree, true) - mid_right);
}

void setup() {
  // Configure serial transport

  if(debugging) Serial.begin(115200);
  else Serial.begin(460800);

  Serial1.begin(115200);

  wing_left.begin();
  wing_right.begin();
  delay(1000);
  wing_left.setPosition(wing_left.degToSignal(0));
  delay(500);
  wing_right.setPosition(wing_right.degToSignal(0));
  delay(500);

  pinMode(22, INPUT);
  pinMode(23, INPUT);
  // pinMode(21, OUTPUT);

  power_left.begin();
  power_right.begin();
  power_source.begin();
  
  encoder_left.begin();
  encoder_right.begin();

  p_wing_left_raw = (wing_raw_data *)malloc(sizeof(wing_raw_data));
  p_wing_right_raw = (wing_raw_data *)malloc(sizeof(wing_raw_data));

  p_wing_left_raw->flag = 0;
  p_wing_right_raw->flag = 0;
  
  delay(100);
  power_left.linearCalibrate(ina219Reading_mA, extMeterReading_mA); delay(100);
  power_right.linearCalibrate(ina219Reading_mA, extMeterReading_mA);  delay(100);
  power_source.linearCalibrate(ina219Reading_mA, extMeterReading_mA); delay(100);

  while(!Serial);
  delay(5000);
  sensor.begin(sensorHandler, 5000);
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

    previous_time = current_time;
    // Serial.println(robot._flappingParam->frequency);
  }

}

void serialEvent(){

  while(Serial.available() > 5){


    for(int i = 0; i < sizeof(serial_data); i++) serial_data[i] = Serial.read();


    if(serial_data[0] == 0xFF && serial_data[6] == 0xEE){
      robot._flappingParam->frequency = (float) serial_data[1] * 0.1f;
      robot._flappingParam->pattern = (uint8_t) serial_data[2];
      robot._flappingParam->offset = ((int8_t) serial_data[3]) - 100;
      robot._flappingParam->amplitude = (uint8_t)serial_data[4];
      robot._flappingParam->down_stroke_percentage = (uint8_t) serial_data[5];

    }

    Serial.flush();

  }
}