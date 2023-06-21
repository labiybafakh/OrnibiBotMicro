#ifndef ORNIBIBOT_HPP
#define ORNIBIBOT_HPP

#include <Arduino.h>
#include <iostream>
#include <math.h>
#include <string.h>
#include "SBUS.hpp"

#define sine 1
#define square 2
#define saw 3
#define rev_saw 4
#define triangle 5

class OrnibiBot
{
private:
  double scalar = 1;

  struct tailPosition
  {
    unsigned int roll;
    unsigned int pitch;
  };

  typedef struct {
    uint8_t offset;
    uint8_t time;
    uint8_t amplitude;
    float frequency;
  } flappingParameter;

  typedef struct{
    volatile float actual_right;
    volatile float actual_left;
    volatile uint16_t desired_right;
    volatile uint16_t desired_left;
  } wingPosition;

  typedef struct{
    volatile float current_left;
    volatile float current_right;
    volatile float voltage_left;
    volatile float voltage_right;
  } wingPower;

  volatile int16_t sineFlap();
  volatile int16_t squareFlap();
  volatile int16_t sawFlap();
  volatile int16_t triangleFlap();
  volatile int16_t reverse_sawFlap();
  // uint32_t getRawPosition(uint8_t pin);
  // double getPositioninRadians(uint8_t pin);
  // double getPositioninDegrees(uint8_t pin);

  volatile uint16_t _periode;
  volatile uint16_t _flapping;

public:
  flappingParameter *_flappingParam;
  wingPosition *_wingPosition;
  wingPower *_wingPower;

  tailPosition tail_position;

  OrnibiBot();
  ~OrnibiBot();
  volatile uint16_t getFlapMs();
  volatile int8_t flappingPattern(uint8_t pattern);


};

#endif