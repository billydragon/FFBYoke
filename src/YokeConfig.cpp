#include "YokeConfig.h"

YokeConfig::YokeConfig() {
  SetDefault();
}

YokeConfig::~YokeConfig() {
  // Auto-generated destructor stub
}

void YokeConfig::SetDefault() {
  TotalGain[0] = TOTALGAIN_X;
  configCPR_X = ENCODER_PR_X * 4;
  configMaxAngle_X = (uint16_t) MAXANGLE_X;
  configInverted_X = false;
  configResetEncoderPosition_X = false;
 
  TotalGain[1] = TOTALGAIN_Y;
  configCPR_Y = ENCODER_PR_Y *4;
  configMaxAngle_Y = (uint16_t) MAXANGLE_Y;
  configInverted_Y = false;
  configResetEncoderPosition_Y = false;

}