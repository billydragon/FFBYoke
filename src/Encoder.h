#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include "YokeConfig.h"


struct Stat{
    bool resetPosition;
    volatile bool currentPinA;
    volatile bool lastPinA;
    volatile bool currentPinB;
    volatile bool lastPinB;
    volatile bool oldState;
    
};


class Encoder {
  public:
    Encoder(void);
    ~Encoder(void);
    Axis axis[2];
    void setConfig(YokeConfig yokeConfig);
    void initVariables(void);
    void updatePosition(int idx);
    
    void tick_X(void);
    void tick_Y(void);

  private:
   
    Stat stat[2];
    int8_t parsePosition(uint8_t axis_num );
    
};

#endif
