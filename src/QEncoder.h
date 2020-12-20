#ifndef QENCODER_h
#define QENCODER_h

#include <Arduino.h>
#include "YokeConfig.h"


class QEncoder {
  public:
    QEncoder(void);
    ~QEncoder(void);
    Axis axis[2];
    void updatePosition(int idx);
    void initVariables(void);
  
};



#endif