#ifndef PWM_H
#define PWM_H
#include <Arduino.h>

#define PWM9   OCR1A
#define PWM10  OCR1B
#define PWM11  OCR1C
#define PWM_FREQ 20000.0f
#define MAXFORCE (F_CPU/(PWM_FREQ*2)) //16000000 is system clock of Leonardo
#define MINFORCE (-MAXFORCE)

#define Dir_X       8
#define Dir_Y       11
#define SERVO_ON_X         5
#define SERVO_ON_Y         6

class Pwm {
 public:   
   Pwm(void);
   ~Pwm(void);
   void begin();
   void setPWM_X(int16_t forces);
   void setPWM_Y(int16_t forces);
   void servo_on_X();
   void servo_on_Y();
   void servo_off_X();
   void servo_off_Y();
};

#endif
