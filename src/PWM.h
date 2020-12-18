#ifndef _PWM_H
#define _PWM_H
#include <Arduino.h>
#include "YokeConfig.h"


class _Pwm {
 public:   
   _Pwm(void);
   ~_Pwm(void);
   void begin();
   void setPWM(int idx, int16_t forces);
   void servo_on(int idx);
   void servo_off(int idx);
};

#endif
