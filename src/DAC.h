#ifndef _DAC_H
#define _DAC_H

#include <Arduino.h>
#include "YokeConfig.h"

#include "DAC8562_DUE.h"

#define DEFAULT_VREF      3.3383
#define DAC_MIN           512 
#define DAC_MAX           65024
#define DEFAULT_CS_PIN    10  

class DAC 
{
 public:   
   
   DAC(void);
   DAC(int cs);
   DAC(int cs, int vref);
   ~DAC(void);
   void begin();
   void setPWM(int idx, int16_t val);
   void servo_on(int idx);
   void servo_off(int idx);

   private:
     volatile int _cs = 10;    //PIN control  SYNC on DAC8563 Board
     volatile int _vref;
};



#endif