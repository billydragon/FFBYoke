#include <Arduino.h>
#include "Joystick.h"
#include "DigitalWriteFast.h"
#include "YokeConfig.h"

#include "PID_V2.h"
//#include "DAC8562.h"
#include "PWM.h"

#ifdef _VARIANT_ARDUINO_DUE_X_
#define Serial  SerialUSB
#include "Due_QDEC.h"

Due_QDEC encoder; 
#else
#include "Encoder.h"
Encoder encoder; 
#endif


_Pwm pwm;
YokeConfig yokeConfig;

int32_t xy_force[2] = {0,0};
int32_t last_xy_force[2] = {0,0};

double Setpoint[2], Input[2], Output[2];
//double Kp=2, Ki=5, Kd=1;
//double aggKp=4, aggKi=0.2, aggKd=1;
double Kp[2] = {KP,KP};
double Ki[2] = {KI,KI};
double Kd[2] = {KD,KD};
PID myPID_X(&Input[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT);
PID myPID_Y(&Input[1], &Output[1], &Setpoint[1], Kp[1], Ki[1], Kd[1], DIRECT);

long debouncing_time = DEBOUNCE_TIME; //Debouncing Time in Milliseconds

bool initialRun = true;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
8, 0, // Button Count, Hat Switch Count
true, true, false, // X and Y, but no Z Axis
true, true, false, // No Rx, Ry, or Rz
false, false, // No rudder or throttle
false, false, false); // No accelerator, brake, or steering

BUTTONS Buttons[8]; 

Gains gain[2];
EffectParams effects[2];

void calculateEncoderPostion(int idx);
void YokeSetGains();
void GetForces();
void gotoPosition(int idx, int32_t targetPosition);
void CalculateMaxSpeedAndMaxAcceleration(int idx);
void findCenter(int idx);
void Push_Button_01_ISR();
void Update_Joystick_Buttons();
#ifndef _VARIANT_ARDUINO_DUE_X_
void calculateEncoderPostion_X();
void calculateEncoderPostion_Y();
#endif

void setup() {
  // put your setup code here, to run once:
  
  pinMode(ANALOG_RX,INPUT);
  pinMode(ANALOG_RY,INPUT);
  Buttons[0].pinNumber = PUSH_BUTTON_01;
  Buttons[0].CurrentState = HIGH;
  Buttons[0].LastState = HIGH;
  Buttons[0].millis_time = millis();
  pinMode(Buttons[0].pinNumber,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Buttons[0].pinNumber), Push_Button_01_ISR, CHANGE);


  #ifdef _VARIANT_ARDUINO_DUE_X_

  analogReadResolution(12);
  #else
  attachInterrupt(digitalPinToInterrupt(encoderPin_XA), calculateEncoderPostion_X, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin_XB), calculateEncoderPostion_X, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(encoderPin_YA), calculateEncoderPostion_Y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin_YB), calculateEncoderPostion_Y, CHANGE);  
  #endif

  encoder.setConfig(yokeConfig);
  Joystick.setRxAxisRange(0,ADC_SCALE);
  Joystick.setRyAxisRange(0,ADC_SCALE);
  
  pwm.begin();
  pwm.setPWM(0,0);  
  pwm.setPWM(1,0);
  pwm.servo_off(0);
  pwm.servo_off(1);
  
  
  Input[0] = encoder.axis[0].currentPosition;
  Input[1] = encoder.axis[1].currentPosition;
  myPID_X.SetMode(AUTOMATIC);
  myPID_X.SetSampleTime(1);
  myPID_X.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  myPID_Y.SetMode(AUTOMATIC);
  myPID_Y.SetSampleTime(1);
  myPID_Y.SetOutputLimits(-PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  Serial.begin(BAUD_RATE);

  Joystick.begin(true);
  YokeSetGains();
  
  
}

void loop() {
  if (initialRun == true ) {
//    position control is not correctly, wheel runs over disired postion serveral times before stop
    
    findCenter(0);
    delay(1000);
    findCenter(1);
    initialRun = false;
  } else

  {
    encoder.updatePosition(0);

    if (encoder.axis[0].currentPosition > encoder.axis[0].maxValue) {
      Joystick.setXAxis(encoder.axis[0].maxValue);
    } else if (encoder.axis[0].currentPosition < encoder.axis[0].minValue) {
      Joystick.setXAxis(encoder.axis[0].minValue);
    } else {
      Joystick.setXAxis(encoder.axis[0].currentPosition);
    }

    encoder.updatePosition(1);
    if (encoder.axis[1].currentPosition > encoder.axis[1].maxValue) {
      Joystick.setYAxis(encoder.axis[1].maxValue);
    } else if (encoder.axis[1].currentPosition < encoder.axis[1].minValue) {
      Joystick.setYAxis(encoder.axis[1].minValue);
    } else {
      Joystick.setYAxis(encoder.axis[1].currentPosition);
        
    }


    GetForces();
  
    xy_force[0] = constrain(xy_force[0], -255, 255);
    xy_force[1] = constrain(xy_force[1], -255, 255);
    
    if (encoder.axis[0].currentPosition >= encoder.axis[0].maxValue) {
      xy_force[0] = 255;
    } else if (encoder.axis[0].currentPosition <= encoder.axis[0].minValue) {
      xy_force[0] = -255;
    }
    if (encoder.axis[1].currentPosition >= encoder.axis[1].maxValue) {
      xy_force[1] = 255;
    } else if (encoder.axis[1].currentPosition <= encoder.axis[1].minValue) {
      xy_force[1] = -255;
    }
      
  }

  pwm.setPWM(0,xy_force[0]);
  pwm.setPWM(1,xy_force[1]);
  Joystick.setRxAxis(analogRead(ANALOG_RX));
  Joystick.setRyAxis(analogRead(ANALOG_RY));
  Update_Joystick_Buttons();
   
}


void Update_Joystick_Buttons()
{

      Joystick.setButton(0, !Buttons[0].CurrentState);
     

}

void GetForces()
{
effects[0].springMaxPosition = encoder.axis[0].maxValue/2;
effects[1].springMaxPosition = encoder.axis[1].maxValue/2;
effects[0].frictionMaxPositionChange = encoder.axis[0].lastPosition - encoder.axis[0].currentPosition;
effects[1].frictionMaxPositionChange = encoder.axis[1].maxValue;
effects[0].inertiaMaxAcceleration = encoder.axis[0].maxValue;
effects[1].inertiaMaxAcceleration = 100;
effects[0].damperMaxVelocity = 100;
effects[1].damperMaxVelocity = 100;

effects[0].springPosition = encoder.axis[0].currentPosition;
effects[1].springPosition = encoder.axis[1].currentPosition;
effects[0].frictionPositionChange = encoder.axis[0].lastPosition - encoder.axis[0].currentPosition; //lastX - posX;
effects[1].frictionPositionChange = encoder.axis[1].lastPosition - encoder.axis[1].currentPosition; //lastY - posY;
effects[0].inertiaAcceleration = 100;
effects[1].inertiaAcceleration = 100;
effects[0].damperVelocity=100;
effects[1].damperVelocity=100;

Joystick.setEffectParams(effects);
Joystick.getForce(xy_force);

}   

void YokeSetGains()
{
//set x axis gains
gain[0].totalGain = TOTALGAIN_X;
gain[0].constantGain = 100;
gain[0].rampGain = 100;
gain[0].squareGain = 100;
gain[0].sineGain = 100;
gain[0].triangleGain = 100;
gain[0].sawtoothdownGain = 100;
gain[0].sawtoothupGain = 100;
gain[0].springGain = 100;
gain[0].damperGain = 100;
gain[0].inertiaGain = 100;
gain[0].frictionGain = 100;

//set y axis gains
gain[1].totalGain = TOTALGAIN_Y;
gain[1].constantGain = 100;
gain[1].rampGain = 100;
gain[1].squareGain = 100;
gain[1].sineGain = 100;
gain[1].triangleGain = 100;
gain[1].sawtoothdownGain = 100;
gain[1].sawtoothupGain = 100;
gain[1].springGain = 100;
gain[1].damperGain = 100;
gain[1].inertiaGain = 100;
gain[1].frictionGain = 100;

Joystick.setGains(gain);

}

#ifndef _VARIANT_ARDUINO_DUE_X_
void calculateEncoderPostion_X() {
  encoder.tick_X();
}

void calculateEncoderPostion_Y() {
  encoder.tick_Y();
}
#endif

void Push_Button_01_ISR()
{
  int bState = digitalReadFast(Buttons[0].pinNumber);
  if(Buttons[0].LastState != bState )
  {
     if((long)(millis() - Buttons[0].millis_time) > debouncing_time ) {

        Buttons[0].CurrentState = bState;
        Buttons[0].LastState = Buttons[0].CurrentState; 
        Buttons[0].millis_time = millis();
    }
  }
      
}

void AutoCalibration(uint8_t idx)
{
    
    if (encoder.axis[idx].currentPosition < encoder.axis[idx].minValue)
    {
      encoder.axis[idx].minValue = encoder.axis[idx].currentPosition;
    }
    if (encoder.axis[idx].currentPosition > encoder.axis[idx].maxValue)
    {
      encoder.axis[idx].maxValue = encoder.axis[idx].currentPosition;
    }

}

void Reset_Encoder(int idx)
{
     #ifdef _VARIANT_ARDUINO_DUE_X_
      encoder.Reset_Encoder(idx);
    #endif
      encoder.axis[idx].currentPosition=0;

      encoder.updatePosition(idx);

}

void findCenter(int idx)
{
  char buff[48];
  int32_t LastPos=0, Axis_Center=0 ,Axis_Range =0;
 
  encoder.axis[idx].minValue =0;
  encoder.axis[idx].maxValue =0;
  Reset_Encoder(idx);

  pwm.servo_on(idx);
   delay(2000);
  Serial.println("Move Axis to Min and Max. Press Button to Finish.");
  while (Buttons[0].CurrentState)
  {
    encoder.updatePosition(idx);
    if(LastPos != encoder.axis[idx].currentPosition)
    {
    AutoCalibration(idx);
    sprintf(buff,"Axis[%d]: %ld,%ld", idx, encoder.axis[idx].minValue, encoder.axis[idx].maxValue);
    Serial.println(buff);
    }
  }
    Axis_Center= (encoder.axis[idx].minValue + encoder.axis[idx].maxValue)/2;
    Axis_Range =  abs(encoder.axis[idx].minValue) + abs(encoder.axis[idx].maxValue);
    encoder.axis[idx].maxValue = Axis_Range/2 -20;
    encoder.axis[idx].minValue = -encoder.axis[idx].maxValue;
    gotoPosition(idx, Axis_Center);    //goto center X
    Reset_Encoder(idx);
    sprintf(buff,"Set Axis[%d]: %ld - 0 - %ld", idx, encoder.axis[idx].minValue, encoder.axis[idx].maxValue);
    Serial.println(buff);
    switch (idx)
    {
    case 0:
      Joystick.setXAxisRange(encoder.axis[idx].minValue, encoder.axis[idx].maxValue);
      Joystick.setXAxis(encoder.axis[idx].currentPosition);
      break;
    case 1:
      Joystick.setYAxisRange(encoder.axis[idx].minValue, encoder.axis[idx].maxValue);
      Joystick.setYAxis(encoder.axis[idx].currentPosition);
      break;
    default:
      break;
    }
   
   
    pwm.setPWM(idx, 0);
}


void gotoPosition(int idx, int32_t targetPosition) {
  char buff[64];
  Setpoint[idx] = targetPosition;
  while (encoder.axis[idx].currentPosition != targetPosition) {
    Setpoint[idx] = targetPosition;
    encoder.updatePosition(idx);
    Input[idx] = encoder.axis[idx].currentPosition ;
    switch (idx)
    {
    case 0:
     myPID_X.Compute();
    break;
    case 1:
     myPID_Y.Compute();
    break;
     default:
      break;
    }
   
    pwm.setPWM(idx, -Output[idx]);
    CalculateMaxSpeedAndMaxAcceleration(idx);
    sprintf(buff,"Axis[%d] Possition: %ld : Target: %ld : Force: %d",idx,encoder.axis[idx].currentPosition, (int32_t)Setpoint[idx], (int)Output[idx] );
    Serial.println(buff);
  }
  
}

void CalculateMaxSpeedAndMaxAcceleration(int idx) {
  if (encoder.axis[idx].maxVelocity < abs(encoder.axis[idx].currentVelocity)) {
    encoder.axis[idx].maxVelocity = abs(encoder.axis[idx].currentVelocity);
  }
  if (encoder.axis[idx].maxAcceleration < abs(encoder.axis[idx].currentAcceleration)) {
    encoder.axis[idx].maxAcceleration = abs(encoder.axis[idx].currentAcceleration);
  }
  if (encoder.axis[idx].maxPositionChange < abs(encoder.axis[idx].positionChange)) {
    encoder.axis[idx].maxPositionChange = abs(encoder.axis[idx].positionChange);
  }
}
