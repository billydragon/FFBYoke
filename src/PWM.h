#ifndef _PWM_H
#define _PWM_H
#include <Arduino.h>
#include "YokeConfig.h"


class _Pwm {
 public:   
   _Pwm(void);
   ~_Pwm(void);
   void begin();
   void setPWM_X(int16_t forces);
   void setPWM_Y(int16_t forces);
   void servo_on_X();
   void servo_on_Y();
   void servo_off_X();
   void servo_off_Y();
};

#endif
