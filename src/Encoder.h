#ifndef ENCODER_h
#define ENCODER_h

#include <Arduino.h>
#include "YokeConfig.h"

#define interrupt_XA  0
#define interrupt_XB  1
#define interrupt_YA  2
#define interrupt_YB  3

#define encoderPin_XA 0
#define encoderPin_XB 1
#define encoderPin_YA 2
#define encoderPin_YB 3

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
    volatile int8_t oldState;

};



class Encoder {
  public:
    Encoder(void);
    ~Encoder(void);
    Axis axis[2];
    void setConfig(YokeConfig yokeConfig);
    void initVariables(void);
    void updatePosition(void);

    void tick_X(void);
    void tick_Y(void);

  private:
   
    Stat stat[2];
    int8_t parsePosition(uint8_t axis_num );
    
};

#endif
