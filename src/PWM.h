#ifndef PWM_H
#define PWM_H
#include <Arduino.h>


#ifdef _VARIANT_ARDUINO_DUE_X_

#define PWM_X       DAC0
#define PWM_Y       DAC1
#define Dir_X       8
#define Dir_Y       11
#define SERVO_ON_X  5
#define SERVO_ON_Y  6
#define DAC_SCALE   4095
#else
#define PWM_X  OCR1A
#define PWM_Y  OCR1B
#define PWM_FREQ 20000.0f
#define MAXFORCE (F_CPU/(PWM_FREQ*2)) //16000000 is system clock of Leonardo
#define MINFORCE (-MAXFORCE)

#define Dir_X       8
#define Dir_Y       11
#define SERVO_ON_X         5
#define SERVO_ON_Y         6
#endif

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
