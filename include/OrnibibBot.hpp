#ifndef ORNIBIBOT_HPP
#define ORNIBIBOT_HPP

#include <Arduino.h>
#include <iostream>
#include <math.h>
#include <string.h>

#define sine 1
#define square 2
#define saw 3
#define rev_saw 4
#define triangle 5

class OrnibiBot
{
private:
  double scalar = 1;

  volatile uint16_t getFlapMs();
  
  struct tailPosition
  {
    unsigned int roll;
    unsigned int pitch;
  };

  struct flappingParameter{
    uint8_t offset;
    uint8_t time;
    uint8_t amplitude;
    uint8_t frequency;
  };

  volatile int16_t sineFlap();
  volatile int16_t squareFlap();
  volatile int16_t sawFlap();
  volatile int16_t triangleFlap();
  volatile int16_t reverse_sawFlap();
  // uint32_t getRawPosition(uint8_t pin);
  // double getPositioninRadians(uint8_t pin);
  // double getPositioninDegrees(uint8_t pin);

  flappingParameter flappingParam;
  flappingParameter *_flappingParam = &flappingParam;



public:
  volatile int16_t _offset;
  volatile uint16_t _time;
  volatile uint16_t _amplitude;
  volatile double _flapFreq;
  volatile uint16_t _periode;
  volatile uint16_t _flapping;

  tailPosition tail_position;
  void flaps(float frequency, uint8_t time);
  volatile int16_t flappingPattern(uint8_t pattern);


};

#endif