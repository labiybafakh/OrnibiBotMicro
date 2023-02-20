#include "OrnibibBot.hpp"
#include <Arduino.h>


volatile uint16_t OrnibiBot::getFlapMs(){
  _periode =  (1000/_flapFreq);
  return _periode;
}

volatile int16_t OrnibiBot::sineFlap(){
    return (volatile int16_t) (_amplitude * sin(((2*M_PI)/(double)OrnibiBot::getFlapMs() * _time))) + _offset;
}

volatile int16_t OrnibiBot::squareFlap(){
    double signal = _amplitude * sin(((2*M_PI)/(double)OrnibiBot::getFlapMs() * _time)) + _offset;
    
    if(signal>0) return _amplitude + _offset;
    else if(signal==0) return (int)0;
    else return _amplitude*-1 + _offset;
}

volatile int16_t OrnibiBot::sawFlap(){
    return (2*_amplitude/M_PI) * atan(tan((M_PI*_time)/(double)OrnibiBot::getFlapMs())) + _offset;
}

volatile int16_t OrnibiBot::reverse_sawFlap(){
    return -(2*_amplitude/M_PI) * atan(tan((M_PI*_time)/(double)OrnibiBot::getFlapMs())) + _offset;

}

volatile int16_t OrnibiBot::triangleFlap(){
    return (2*_amplitude/M_PI) * asin(sin((2*M_PI/(double)OrnibiBot::getFlapMs())*_time)) + _offset;
}

volatile int16_t OrnibiBot::flappingPattern(uint8_t pattern){
    if(pattern==sine) OrnibiBot::sineFlap();
    else if(pattern==triangle) OrnibiBot::triangleFlap();
    else if(pattern==square) OrnibiBot::squareFlap();
    else if(pattern==saw) OrnibiBot::sawFlap();
    else if(pattern==rev_saw) OrnibiBot::reverse_sawFlap();
}

double OrnibiBot::getRawPosition(uint8_t pin){
    pinMode(pin, INPUT);
    return pulseIn(pin, RISSING, 1000);
}

double OrnibiBot::getPositioninRadians(uint8_t pin){
    return OrnibiBot::getRawPosition * scalar;
}

double OrnibiBot::getPositioninDegrees(uint8_t pin){
    return OrnibiBot::getPositioninRadians(pin) * RADS;
}