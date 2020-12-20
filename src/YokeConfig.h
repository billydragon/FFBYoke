#ifndef YOKECONFIG_h
#define YOKECONFIG_h
#include <Arduino.h>

#define BAUD_RATE 115200

#define MAX_X_VALUE         3000
#define MAX_Y_VALUE         2000
#define TOTALGAIN_X         100
#define TOTALGAIN_Y         100

#ifdef _VARIANT_ARDUINO_DUE_X_

#define encoderPin_XA       2
#define encoderPin_XB       13
#define encoderPin_YA       5
#define encoderPin_YB       4

#define PUSH_BUTTON_01       A5

#define PWM_X               DAC0
#define PWM_Y               DAC1
#define Dir_X               28
#define Dir_Y               29
#define SERVO_ON_X          30
#define SERVO_ON_Y          31
#define DUE_DAC_SCALE       4095
#define DUE_ADC_SCALE       4095

#else

#define encoderPin_XA       0
#define encoderPin_XB       1
#define encoderPin_YA       2
#define encoderPin_YB       3


#define PUSH_BUTTON_01        7

#define PWM_X  9
#define PWM_Y  10
#define PWM_FREQ 20000.0f
#define MAXFORCE (F_CPU/(PWM_FREQ*2)) //16000000 is system clock of Leonardo
#define MINFORCE (-MAXFORCE)

#define Dir_X       8
#define Dir_Y       11
#define SERVO_ON_X         5
#define SERVO_ON_Y         6
#define DAC_SCALE           1023
#define ADC_SCALE           1023
#endif

#define ANALOG_RX           A0
#define ANALOG_RY           A1
#define DEBOUNCE_TIME       50

#define PID_OUTPUT_LIMIT    80
#define KP      2
#define KI      0.5
#define KD      0.05

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


struct BUTTONS{
  
    volatile uint8_t pinNumber;
    volatile int CurrentState;
    volatile int LastState;
    volatile uint32_t millis_time;
};


class YokeConfig {
  public:
    
        YokeConfig(void);
        void SetDefaults(void);
        ~YokeConfig(void);
};

#endif