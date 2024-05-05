#include "OrnibibBot.hpp"
#include <Arduino.h>

OrnibiBot::OrnibiBot(){
    _flappingParam = (flappingParameter *)malloc(sizeof(flappingParameter));
    p_wing_data = (wingData *)malloc(sizeof(wingData));
}

OrnibiBot::~OrnibiBot(){
    free(_flappingParam);
    free(p_wing_data);
}

volatile uint16_t OrnibiBot::getFlapMs(){
    //Calculate the periode of signal
  _periode =  (1000/_flappingParam->frequency);
  return _periode;
}

volatile int8_t OrnibiBot::sineFlap(){
    return (volatile int8_t) (_flappingParam->amplitude * sin(((2*M_PI)/(double)OrnibiBot::getFlapMs() * _flappingParam->time))) + _flappingParam->offset;
}

volatile int8_t OrnibiBot::sineFlapWithAdjust(uint8_t down_stroke_periode){

    double down_stroke_periode_ = (double)OrnibiBot::getFlapMs() * (down_stroke_periode * 0.01f);

    if(_flappingParam->time >= 0 && _flappingParam->time < (uint16_t)down_stroke_periode_)
        return (volatile int8_t) (_flappingParam->amplitude * cos(M_PI * _flappingParam->time /down_stroke_periode_));

    else{
        return (volatile int8_t) (_flappingParam->amplitude * -cos(M_PI * (_flappingParam->time - down_stroke_periode_) / ((double)OrnibiBot::getFlapMs()-down_stroke_periode_)));
    }
}

volatile int8_t OrnibiBot::squareFlap(){
    uint8_t signal =  (uint8_t) _flappingParam->amplitude * sin(((2*M_PI)/(double)OrnibiBot::getFlapMs() * _flappingParam->time));
    
    if(signal>0) return (volatile int8_t) _flappingParam->amplitude + _flappingParam->offset;
    // else if(signal==0) return (volatile int8_t)0 + _flappingParam->offset;
    else return (volatile int8_t) _flappingParam->amplitude*-1 + _flappingParam->offset;
}

volatile int8_t OrnibiBot::sawFlap(){
    return (volatile int8_t) (2*_flappingParam->amplitude/M_PI) * atan(tan((M_PI*_flappingParam->time)/(double)OrnibiBot::getFlapMs())) + _flappingParam->offset;
}

volatile int8_t OrnibiBot::reverse_sawFlap(){
    return (volatile int8_t) -(2*_flappingParam->amplitude/M_PI) * atan(tan((M_PI*_flappingParam->time)/(double)OrnibiBot::getFlapMs())) + _flappingParam->offset;

}

volatile int8_t OrnibiBot::triangleFlap(){
    return (volatile int8_t)(2*_flappingParam->amplitude/M_PI) * asin(sin((2*M_PI/(double)OrnibiBot::getFlapMs())*_flappingParam->time)) + _flappingParam->offset;
}


volatile int8_t OrnibiBot::flappingPattern(uint8_t pattern, uint8_t down_stroke_percentage){
    if(pattern==sine) return OrnibiBot::sineFlap();
    else if(pattern==triangle) return OrnibiBot::triangleFlap();
    else if(pattern==square) return OrnibiBot::squareFlap();
    else if(pattern==saw) return OrnibiBot::sawFlap();
    else if(pattern==reverse_saw) return OrnibiBot::reverse_sawFlap();
    else if(pattern==adjust_sine) return OrnibiBot::sineFlapWithAdjust(down_stroke_percentage);
}
