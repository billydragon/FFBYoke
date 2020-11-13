#include "Encoder.h"
#include "DigitalWriteFast.h"

Encoder::Encoder() {
  pinMode (encoderPin_XA, INPUT_PULLUP);
  pinMode (encoderPin_XB, INPUT_PULLUP);
  pinMode (encoderPin_YA, INPUT_PULLUP);
  pinMode (encoderPin_YB, INPUT_PULLUP);
}

Encoder::~Encoder() {
}

void Encoder::setConfig(YokeConfig YokeConfig) {
  axis[0].cPR = YokeConfig.configCPR_X ;
  axis[0].maxAngle = YokeConfig.configMaxAngle_X;
  axis[0].inverted = YokeConfig.configInverted_X;
  axis[0].maxValue = (float)axis[0].maxAngle / 2 / 360 * axis[0].cPR ;
  axis[0].minValue =  - axis[0].maxValue;
  stat[0].resetPosition = YokeConfig.configResetEncoderPosition_X;

  axis[1].cPR = YokeConfig.configCPR_Y ;
  axis[1].maxAngle = YokeConfig.configMaxAngle_Y;
  axis[1].inverted = YokeConfig.configInverted_Y;
  axis[1].maxValue = (float)axis[1].maxAngle / 2 / 360 * axis[1].cPR ;
  axis[1].minValue =  -axis[1].maxValue;
  stat[1].resetPosition = YokeConfig.configResetEncoderPosition_Y;

  initVariables();
}

void Encoder::initVariables() {
  for (int i=0;i<2;i++)
  {
      axis[i].currentPosition = 0;
      axis[i].lastPosition = 0;
      axis[i].correctPosition = 0;
      axis[i].maxAcceleration = 0;
      axis[i].maxVelocity = 0;
      axis[i].lastEncoderTime = (uint32_t) millis();
      axis[i].lastVelocity = 0;
      stat[i].currentPinA = LOW;
      stat[i].currentPinB = LOW;
      stat[i].lastPinA = LOW;
      stat[i].lastPinB = LOW; 
  }
}

void  Encoder::updatePosition() {

  /*  
  if (usePinZ) {
    currentPinZ = digitalReadFast(encoderPinZ);
    if (z1stUp) {
      correctPosition = correctPosition; //found correct position
      z1stUp = true;
    } else {
      if (currentPosition > correctPosition * 0.05 || currentPosition < correctPosition * 0.05  ) {
        currentPosition = correctPosition;
      }
    }
  } */
      for (int i=0;i<2;i++)
      {
        axis[i].positionChange = axis[i].currentPosition - axis[i].lastPosition;
        uint32_t currentEncoderTime = (int32_t) millis();
        int16_t diffTime = (int16_t)(currentEncoderTime - axis[i].lastEncoderTime) ;
        if (diffTime > 0) {
          axis[i].currentVelocity = axis[i].positionChange / diffTime;
          axis[i].currentAcceleration = (abs(axis[i].currentVelocity) - abs(axis[i].lastVelocity)) / diffTime;
          axis[i].lastEncoderTime = currentEncoderTime;
          axis[i].lastVelocity = axis[i].currentVelocity;
        }
        axis[i].lastPosition = axis[i].currentPosition;
      }
}

int8_t Encoder::parsePosition(uint8_t axis_num ) { //4 state
  if (stat[axis_num].lastPinA && stat[axis_num].lastPinB) {
    if (!stat[axis_num].currentPinA && stat[axis_num].currentPinB) return 1;
    if (stat[axis_num].currentPinA && !stat[axis_num].currentPinB) return -1;
  }
  else if (!stat[axis_num].lastPinA && stat[axis_num].lastPinB) {
    if (!stat[axis_num].currentPinA && !stat[axis_num].currentPinB) return 1;
    if (stat[axis_num].currentPinA && stat[axis_num].currentPinB) return -1;
  }
  else if (!stat[axis_num].lastPinA && !stat[axis_num].lastPinB) {
    if (stat[axis_num].currentPinA && !stat[axis_num].currentPinB) return 1;
    if (!stat[axis_num].currentPinA && stat[axis_num].currentPinB) return -1;
  }
  else if (stat[axis_num].lastPinA && !stat[axis_num].lastPinB) {
    if (stat[axis_num].currentPinA && stat[axis_num].currentPinB) return 1;
    if (!stat[axis_num].currentPinA && !stat[axis_num].currentPinB) return -1;
  }
}

//this code copy from Rotary Encoder of Matthias Hertel
const int8_t KNOBDIR[] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
  -1,  0,  0,  1,
0,  1, -1,  0  };

void Encoder::tick_X(void)
{
  int sig1 = digitalReadFast(encoderPin_XA);
  int sig2 = digitalReadFast(encoderPin_XB);
  int8_t thisState = sig1 | (sig2 << 1);

  if (stat[0].oldState != thisState) {
    axis[0].currentPosition += KNOBDIR[thisState | (stat[0].oldState<<2)];
    stat[0].oldState = thisState;
  } 
} 

void Encoder::tick_Y(void)
{
  int sig1 = digitalReadFast(encoderPin_YA);
  int sig2 = digitalReadFast(encoderPin_YB);
  int8_t thisState = sig1 | (sig2 << 1);

  if (stat[1].oldState != thisState) {
    axis[1].currentPosition += KNOBDIR[thisState | (stat[1].oldState<<2)];
    stat[1].oldState = thisState;
  } 
} 

