#ifndef YOKECONFIG_h
#define YOKECONFIG_h
#include <Arduino.h>

#define BAUD_RATE 115200

#define ENCODER_PR_X        1000
#define ENCODER_PR_Y        1000
#define MAXANGLE_X          630
#define MAXANGLE_Y          630
#define TOTALGAIN_X         0.5
#define TOTALGAIN_Y         0.5

#ifdef _VARIANT_ARDUINO_DUE_X_

#define encoderPin_XA       22
#define encoderPin_XB       23
#define encoderPin_XZ       24
#define encoderPin_YA       25
#define encoderPin_YB       26
#define encoderPin_YZ       27

#define LIMIT_SWITCH        48

#define PWM_X               DAC0
#define PWM_Y               DAC1
#define Dir_X               28
#define Dir_Y               29
#define SERVO_ON_X          30
#define SERVO_ON_Y          31
#define DAC_SCALE           4095
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
#endif

#define DEBOUNCE_TIME       5050

#define PID_OUTPUT_LIMIT    127
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