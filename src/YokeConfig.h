#ifndef YOKECONFIG_h
#define YOKECONFIG_h
#include <Arduino.h>

#define CS_PIN              52    // SS pin for SPI
#define LDAC_PIN            50                
#define BAUD_RATE 115200
#define XY_FORCE_INVERT    
#define MAX_X_VALUE         3000
#define MAX_Y_VALUE         2000
#define TOTALGAIN_X         50
#define TOTALGAIN_Y         50
#define NUM_OF_BUTTONS      32
#define NUM_OF_HATSWITCH    0
#define AXIS_EDGE_PROTECT   50

#define X_AXIS              0
#define Y_AXIS              1
#ifdef _VARIANT_ARDUINO_DUE_X_

#define encoderPin_XA       2
#define encoderPin_XB       13
#define encoderPin_YA       5
#define encoderPin_YB       4

#define PUSH_BUTTON_01       A5

#define PWM_FREQ1           10000
#define PWM_FREQ2           20000

#define DUE_PWM_SCALE       255
#define PWM_PIN_X           6
#define PWM_PIN_Y           7
#define PWM_X               DAC0
#define PWM_Y               DAC1
#define Dir_X               28
#define Dir_Y               29
#define SERVO_ON_X          30
#define SERVO_ON_Y          31
#define DAC_SCALE       4095
#define ADC_SCALE       4095
#define MOTOR_DIR_DELAY     10;

#else

#define encoderPin_XA    (int)0
#define encoderPin_XB    (int)1
#define encoderPin_YA    (int)2
#define encoderPin_YB    (int)3


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
#define DEBOUNCE_TIME       15

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
        //bool Motor_Inv_X = false;
        //bool Motor_Inv_Y = false;
        //int16_t Motor_DIR_delay;
};

#endif