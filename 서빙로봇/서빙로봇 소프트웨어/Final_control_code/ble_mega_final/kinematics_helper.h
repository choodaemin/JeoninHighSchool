#ifndef _ENCODER_H_
#define _ENCODER_H_
#include"config.h"
#include"QGPMaker_Encoder.h"

 float getWheelRotatialSpeed(QGPMaker_Encoder *enc,long &lastEnc,unsigned long last_time,bool reverse){
    unsigned long curt=millis();
    long curenc=enc->read();
    curenc=reverse?-curenc:curenc;
    float w=(curenc-lastEnc)*resolution/((curt-last_time)/1000.0)/r;
    lastEnc=curenc;
    return w;
    
  }

#endif
