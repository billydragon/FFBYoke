#ifndef YOKECONFIG_h
#define YOKECONFIG_h
#include <Arduino.h>

#define BAUD_RATE 115200

#define ENCODER_PR_X        1000
#define ENCODER_PR_Y        1000
#define MAXANGLE_X          630
#define MAXANGLE_Y          630
#define TOTALGAIN_X          1
#define TOTALGAIN_Y          1
#define LIMIT_SWITCH        4

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