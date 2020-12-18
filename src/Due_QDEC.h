#ifndef DUE_QDEC_h
#define DUE_QDEC_h

#include <Arduino.h>
#include "YokeConfig.h"



const int quadX_A = 2;
const int quadX_B = 13;
const unsigned int mask_quadX_A = digitalPinToBitMask(quadX_A);
const unsigned int mask_quadX_B = digitalPinToBitMask(quadX_B);

const int quadY_A = 5;
const int quadY_B = 4;
const unsigned int mask_quadY_A = digitalPinToBitMask(quadY_A);
const unsigned int mask_quadY_B = digitalPinToBitMask(quadY_B);

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


class Due_QDEC {
  public:
    Due_QDEC(void);
    ~Due_QDEC(void);
    Axis axis[2];
    void setConfig(YokeConfig yokeConfig);
    void initVariables(void);
    void updatePosition_X(void);
    void updatePosition_Y(void);
    void activateCNT_TC0();  
    void activateCNT_TC2();
    void Reset_Encoder_X();
    void Reset_Encoder_Y();

    
};

#endif