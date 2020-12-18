#ifndef YOKECONFIG_h
#define YOKECONFIG_h
#include <Arduino.h>

#define BAUD_RATE 115200

#define ENCODER_PR_X        2000
#define ENCODER_PR_Y        2000
#define MAXANGLE_X          900
#define MAXANGLE_Y          900
#define MAX_X_VALUE         8000
#define MAX_Y_VALUE         6000
#define TOTALGAIN_X         100
#define TOTALGAIN_Y         100


#ifdef _VARIANT_ARDUINO_DUE_X_

#define encoderPin_XA       2
#define encoderPin_XB       13
#define encoderPin_YA       5
#define encoderPin_YB       4

#define LIMIT_SWITCH        A5

#define PWM_X               DAC0
#define PWM_Y               DAC1
#define Dir_X               28
#define Dir_Y               29
#define SERVO_ON_X          30
#define SERVO_ON_Y          31
#define DAC_SCALE           4095
#define ADC_SCALE           4095

#else

#define encoderPin_XA       0
#define encoderPin_XB       1
#define encoderPin_YA       2
#define encoderPin_YB       3


#define LIMIT_SWITCH        7

#define PWM_X  OCR1A
#define PWM_Y  OCR1B
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

#define PID_OUTPUT_LIMIT    255
#define KP      5
#define KI      1
#define KD      0.01

class YokeConfig {
  public:
    
        YokeConfig(void);
        ~YokeConfig(void);
        void SetDefault();
        int16_t TotalGain[2];
        uint32_t configCPR_X;
        uint16_t configMaxAngle_X;    
        bool configInverted_X;
        bool configResetEncoderPosition_X;
        uint32_t configCPR_Y;
        uint16_t configMaxAngle_Y;    
        bool configInverted_Y;
        bool configResetEncoderPosition_Y;
  
};


#endif