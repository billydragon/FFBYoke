#include "QEncoder.h"
#include "Encoder.h"

Encoder Enc[]= {Encoder(encoderPin_XA, encoderPin_XB), Encoder(encoderPin_YA, encoderPin_YB)};

QEncoder::QEncoder() 
{
        initVariables();
}

QEncoder::~QEncoder() {

}

void QEncoder::initVariables() {
  for (int i=0;i<2;i++)
  {
      axis[i].currentPosition = -999;
      axis[i].lastPosition = 0;
      axis[i].correctPosition = 0;
      axis[i].maxAcceleration = 0;
      axis[i].maxVelocity = 0;
      axis[i].lastEncoderTime = (uint32_t) millis();
      axis[i].lastVelocity = 0;
      
  }
}

void  QEncoder::updatePosition(int idx) 
{       
    
        axis[idx].currentPosition = Enc[idx].read();
        if (axis[idx].currentPosition != axis[idx].lastPosition)
        { 
            axis[idx].positionChange = axis[idx].currentPosition - axis[idx].lastPosition;
            uint32_t currentEncoderTime = (int32_t) millis();
            int16_t diffTime = (int16_t)(currentEncoderTime - axis[idx].lastEncoderTime) ;
            if (diffTime > 0) {
                axis[idx].currentVelocity = axis[idx].positionChange / diffTime;
                axis[idx].currentAcceleration = (abs(axis[idx].currentVelocity) - abs(axis[idx].lastVelocity)) / diffTime;
                axis[idx].lastEncoderTime = currentEncoderTime;
                axis[idx].lastVelocity = axis[idx].currentVelocity;
            }
            axis[idx].lastPosition = axis[idx].currentPosition;  
            
        }
}



