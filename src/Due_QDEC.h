#ifndef DUE_QDEC_h
#define DUE_QDEC_h

#include <Arduino.h>
#include "YokeConfig.h"


#ifdef _VARIANT_ARDUINO_DUE_X_
const int quadX_A = 2;
const int quadX_B = 13;
const unsigned int mask_quadX_A = digitalPinToBitMask(quadX_A);
const unsigned int mask_quadX_B = digitalPinToBitMask(quadX_B);

const int quadY_A = 5;
const int quadY_B = 4;
const unsigned int mask_quadY_A = digitalPinToBitMask(quadY_A);
const unsigned int mask_quadY_B = digitalPinToBitMask(quadY_B);


class Due_QDEC {
  public:
    Due_QDEC(void);
    ~Due_QDEC(void);
    Axis axis[2];
    void setConfig();
    void initVariables(void);
    void updatePosition(int idx);
    
    void activateCNT_TC0();  
    void activateCNT_TC2();
    void Reset_Encoder(int idx);

};

#endif
#endif