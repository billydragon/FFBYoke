#ifndef YOKECONFIG_h
#define YOKECONFIG_h
#include <Arduino.h>

#define CS_PIN              10    // SS pin for SPI

#define BAUD_RATE           115200

#define MAX_X_VALUE         3000
#define MAX_Y_VALUE         2000
#define TOTALGAIN_X         100
#define TOTALGAIN_Y         100

#define X_AXIS              0
#define Y_AXIS              1


#define encoderPin_XA       0
#define encoderPin_XB       1
#define encoderPin_YA       2
#define encoderPin_YB       3


#define PUSH_BUTTON_01      7

#define PWM_X               9
#define PWM_Y               10
#define PWM_FREQ            20000.0f
#define MAXFORCE            (F_CPU/(PWM_FREQ*2)) //16000000 is system clock of Leonardo
#define MINFORCE            (-MAXFORCE)

#define Dir_X               8
#define Dir_Y               11
#define SERVO_ON_X          5
#define SERVO_ON_Y          6


#define DEBOUNCE_TIME       50

#define PID_OUTPUT_LIMIT    50
#define PID_SAMPLE_TIME     0.01
#define KP      0.5
#define KI      1
#define KD      0.01

 struct Axis{
   
    int32_t maxValue;
    int32_t  minValue;    
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


#endif