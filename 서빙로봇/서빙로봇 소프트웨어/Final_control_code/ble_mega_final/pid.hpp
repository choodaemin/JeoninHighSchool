#ifndef _PID_HPP_
#define _PID_HPP_


class MPID{
  public:
  QGPMaker_Encoder *enc; 
  float kP,kI,kD;
  int error;
  int currentError;
  int lastError;
  unsigned long lastTime;
  unsigned long currentTime;
  long lastEnc;
  long currentEnc;
  bool reverse;
  long deltaEnc;
  float deltaT;
  bool first;
  MPID(QGPMaker_Encoder *Enc,float kp,float ki,float kd,bool revs){
    enc=Enc;
    kP=kp;kI=ki;kD=kd; 
    error=0;
    currentError=0;
    lastError=0;
    reverse=revs;
    deltaEnc=0;
    deltaT=0;//TODO
    first=true;
  }
  void init(){
    lastTime=0;
    lastEnc=0;
  }
  void tic(){
    currentTime=millis();
    currentEnc=enc->read();
    currentEnc=reverse?-currentEnc:currentEnc;
  }

  void toc(){
    lastTime=currentTime;
    lastEnc=currentEnc;
  }
  int getPWM(int given){
    if(first){
      first=false;
      return 0.0;
    }else{
      deltaEnc=currentEnc-lastEnc;
      deltaT=(currentTime-lastTime)/1000.0;
      float mesPWM=speed2pwm(deltaEnc*resolution/deltaT);
      error=given-mesPWM;
    
      currentError+=kI*error;
      if(given == 0 && error == 0){
        currentError = 0;
        }
      int s=kP*error+currentError+kD*(error-lastError);
      lastError=error;
      return constrain(s,-255,255); 
    }
  }
};
#endif
