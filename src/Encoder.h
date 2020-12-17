#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include "YokeConfig.h"



struct Axis{
    uint32_t cPR;
    uint16_t maxAngle;
    int32_t maxValue;
    int32_t  minValue;    
    bool inverted;
    uint32_t lastEncoderTime;
    int32_t  currentPosition;
    int32_t  lastPosition;
    int32_t  correctPosition;    
    int32_t  currentVelocity;
    int32_t  lastVelocity;
    int32_t  maxVelocity;
    int32_t  currentAcceleration;
    int32_t  maxAcceleration;
    int32_t  positionChange;
    int32_t  maxPositionChange;
   

};

struct Stat{
    bool resetPosition;
    volatile bool currentPinA;
    volatile bool lastPinA;
    volatile bool currentPinB;
    volatile bool lastPinB;
    volatile bool oldState;
    volatile bool currentPinZ;
    volatile bool lastPinZ;
    volatile bool z1stUp = false;

};



class Encoder {
  public:
    Encoder(void);
    ~Encoder(void);
    Axis axis[2];
    void setConfig(YokeConfig yokeConfig);
    void initVariables(void);
    void updatePosition_X(void);
    void updatePosition_Y(void);

    void tick_X(void);
    void tick_Y(void);

  private:
   
    Stat stat[2];
    int8_t parsePosition(uint8_t axis_num );
    
};

#endif
