#include "OrnibibBot.h"



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
    return -(2*_amplitude/M_PI) * atan(tan((M_PI*_time)/(double)OrnibiBot::getFlapMs())) + _offset;
}

volatile int16_t OrnibiBot::triangleFlap(){
    return (2*_amplitude/M_PI) * asin(sin((2*M_PI/(double)OrnibiBot::getFlapMs())*_time)) + _offset;
}
