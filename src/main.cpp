#include <Arduino.h>
#include "Joystick.h"
#include "DigitalWriteFast.h"
#include "YokeConfig.h"
#include "ConfigManager.h"
#include "PID_V2.h"


#ifdef _VARIANT_ARDUINO_DUE_X_
#define Serial  SerialUSB
#endif

#define USING_DAC
#define DUE_QDEC


#ifdef USING_DAC
#include "DAC8563.h"
DAC8563 pwm=DAC8563(CS_PIN);
#else
#include "DuePWM.h"
DuePWM pwm(PWM_FREQ1,PWM_FREQ2);
#endif

#ifdef DUE_QDEC
#include "Due_QDEC.h"
Due_QDEC encoder;
#else
#include "QEncoder.h"
QEncoder encoder; 
#endif


int32_t xy_force[2] = {0,0};
 //int32_t last_xy_force[2] = {0,0};

double Setpoint[2], Input[2], Output[2];
//double Kp=2, Ki=5, Kd=1;
//double aggKp=4, aggKi=0.2, aggKd=1;
 double Kp[2];
 double Ki[2];
 double Kd[2];

PID  myPID[2] = {PID(&Input[X_AXIS], &Output[X_AXIS], &Setpoint[X_AXIS], Kp[X_AXIS], Ki[X_AXIS], Kd[X_AXIS], DIRECT),
                PID(&Input[Y_AXIS], &Output[Y_AXIS], &Setpoint[Y_AXIS], Kp[Y_AXIS], Ki[Y_AXIS], Kd[Y_AXIS], DIRECT)}; 

volatile long debouncing_time = DEBOUNCE_TIME; //Debouncing Time in Milliseconds

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
1, 0, // Button Count, Hat Switch Count
true, true, false, // X and Y, but no Z Axis
false, false, false, // No Rx, Ry, or Rz
false, false, // No rudder or throttle
false, false, false); // No accelerator, brake, or steering

BUTTONS Buttons; 

volatile uint8_t initialRun = 1;

Gains gain[2];
EffectParams effects[2];

ConfigManager CfgManager = ConfigManager();
YokeConfig yokeConfig;
void calculateEncoderPostion(int idx);
void Set_Gains();
void SetEffects();
void gotoPosition(int idx, int32_t targetPosition);
void CalculateMaxSpeedAndMaxAcceleration(int idx);
void findCenter(int axis);
void Push_Button_01_ISR();
void Update_Joystick_Buttons();

//void calculateEncoderPostion_X();
//void calculateEncoderPostion_Y();

void Set_PIDs()
{

  for(int ax =0; ax <2 ; ax++)
  {
  Kp[ax]= CfgManager._Pids[ax].pid.Kp;
  Ki[ax] = CfgManager._Pids[ax].pid.Ki;
  Kd[ax] = CfgManager._Pids[ax].pid.Kd;
  myPID[ax].SetSampleTime(CfgManager._Pids[ax].pid.SampleTime);
  myPID[ax].SetOutputLimits(-CfgManager._Pids[ax].pid.MaxOutput, CfgManager._Pids[ax].pid.MaxOutput);
  myPID[ax].SetMode(AUTOMATIC);
  Input[ax] = encoder.axis[ax].currentPosition;
  }
 
}

void setup() {
 
  Serial.begin(BAUD_RATE);
  Joystick.begin(true);
  CfgManager.begin();  
  Set_PIDs();  
  myPID[0] = PID(&Input[X_AXIS], &Output[X_AXIS], &Setpoint[X_AXIS], Kp[X_AXIS], Ki[X_AXIS], Kd[X_AXIS], DIRECT);
  myPID[1] = PID(&Input[Y_AXIS], &Output[Y_AXIS], &Setpoint[Y_AXIS], Kp[Y_AXIS], Ki[Y_AXIS], Kd[Y_AXIS], DIRECT); 

  Buttons.pinNumber = PUSH_BUTTON_01;
  Buttons.CurrentState = HIGH;
  Buttons.LastState = HIGH;
  Buttons.millis_time = millis();

  pinMode(Buttons.pinNumber,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Buttons.pinNumber), Push_Button_01_ISR, CHANGE);

  /*
  attachInterrupt(digitalPinToInterrupt(encoderPin_XA), calculateEncoderPostion_X, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin_XB), calculateEncoderPostion_X, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(encoderPin_YA), calculateEncoderPostion_Y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin_YB), calculateEncoderPostion_Y, CHANGE); 
  */ 

  delay(200);
  #ifdef USING_DAC
  pwm.begin(CfgManager);
  #else
  //DuePWM::pinFreq1( uint32_t pin ) // pin must be 6 through 9
  DuePWM::pinFreq2(PWM_PIN_X); // pin must be 6 through 9
  DuePWM::pinFreq2(PWM_PIN_Y); // pin must be 6 through 9
  #endif
  delay(200);
  pwm.servo_off(X_AXIS);
  pwm.servo_off(Y_AXIS);
  pwm.setPWM(X_AXIS,0);  
  pwm.setPWM(Y_AXIS,0);
  
  initialRun = CfgManager._SysConfig.Byte.Auto_Calibration;
  if(!Buttons.CurrentState)
   {
     initialRun = 1;
   }
  
}

void loop() {

   CfgManager.GetUpdate();
   Set_PIDs(); 
  
    if (initialRun == 1 ) 
      {
      findCenter(X_AXIS);
      delay(1000);
      findCenter(Y_AXIS);
      initialRun = 0;
      }
  
    encoder.updatePosition(X_AXIS);

    if (encoder.axis[X_AXIS].currentPosition > encoder.axis[X_AXIS].maxValue) {
      Joystick.setXAxis(encoder.axis[X_AXIS].maxValue);
    } else if (encoder.axis[X_AXIS].currentPosition < encoder.axis[X_AXIS].minValue) {
      Joystick.setXAxis(encoder.axis[X_AXIS].minValue);
    } else {
      Joystick.setXAxis(encoder.axis[X_AXIS].currentPosition);
    }

    encoder.updatePosition(Y_AXIS);
    if (encoder.axis[Y_AXIS].currentPosition > encoder.axis[Y_AXIS].maxValue) {
      Joystick.setYAxis(encoder.axis[Y_AXIS].maxValue);
    } else if (encoder.axis[Y_AXIS].currentPosition < encoder.axis[Y_AXIS].minValue) {
      Joystick.setYAxis(encoder.axis[Y_AXIS].minValue);
    } else {
      Joystick.setYAxis(encoder.axis[Y_AXIS].currentPosition);
        
    }

   
    Set_Gains();
    SetEffects();
    Joystick.getForce(xy_force);

    if(CfgManager._SysConfig.Byte.Swap_XY_Force == 1)   //Swap axis forces
    {
        int32_t f = xy_force[Y_AXIS];
        xy_force[Y_AXIS] = xy_force[X_AXIS];
        xy_force[X_AXIS] = f;

    }

    xy_force[X_AXIS] = constrain(xy_force[X_AXIS], -255, 255);
    xy_force[Y_AXIS] = constrain(xy_force[Y_AXIS], -255, 255);
    
    if (encoder.axis[X_AXIS].currentPosition >= encoder.axis[X_AXIS].maxValue) {
      xy_force[X_AXIS] = 255;
    } else if (encoder.axis[X_AXIS].currentPosition <= encoder.axis[X_AXIS].minValue) {
      xy_force[X_AXIS] = -255;
    }
    if (encoder.axis[Y_AXIS].currentPosition >= encoder.axis[Y_AXIS].maxValue) {
      xy_force[Y_AXIS] = 255;
    } else if (encoder.axis[Y_AXIS].currentPosition <= encoder.axis[Y_AXIS].minValue) {
      xy_force[Y_AXIS] = -255;
    }

    pwm.setPWM(X_AXIS,xy_force[X_AXIS]);
    CalculateMaxSpeedAndMaxAcceleration(X_AXIS);
    pwm.setPWM(Y_AXIS,xy_force[Y_AXIS]);
    CalculateMaxSpeedAndMaxAcceleration(Y_AXIS);
    Update_Joystick_Buttons();
  
}


void Update_Joystick_Buttons()
{

    Joystick.setButton(0, !Buttons.CurrentState);
     
}

void SetEffects()
{
effects[X_AXIS].springPosition = encoder.axis[X_AXIS].currentPosition;
effects[Y_AXIS].springPosition = encoder.axis[Y_AXIS].currentPosition;
effects[X_AXIS].springMaxPosition = encoder.axis[X_AXIS].maxValue/2;
effects[Y_AXIS].springMaxPosition = encoder.axis[Y_AXIS].maxValue/2;

effects[X_AXIS].frictionPositionChange = encoder.axis[X_AXIS].positionChange; //lastX - posX;
effects[Y_AXIS].frictionPositionChange = encoder.axis[Y_AXIS].positionChange; //lastY - posY;
effects[X_AXIS].frictionMaxPositionChange = encoder.axis[X_AXIS].maxPositionChange;
effects[Y_AXIS].frictionMaxPositionChange = encoder.axis[Y_AXIS].maxPositionChange;

effects[X_AXIS].inertiaAcceleration = encoder.axis[X_AXIS].currentPosition;;
effects[Y_AXIS].inertiaAcceleration = encoder.axis[Y_AXIS].currentPosition;;
effects[X_AXIS].inertiaMaxAcceleration = encoder.axis[X_AXIS].maxAcceleration;
effects[Y_AXIS].inertiaMaxAcceleration = encoder.axis[Y_AXIS].maxAcceleration;

effects[X_AXIS].damperVelocity=encoder.axis[X_AXIS].currentVelocity;
effects[Y_AXIS].damperVelocity=encoder.axis[Y_AXIS].currentVelocity;
effects[X_AXIS].damperMaxVelocity = encoder.axis[X_AXIS].maxVelocity;
effects[Y_AXIS].damperMaxVelocity = encoder.axis[Y_AXIS].maxVelocity;

Joystick.setEffectParams(effects);

}   

void Set_Gains()
{

for(int i = 0; i < 2 ; i++)
{
          //set x axis gains
          gain[i].totalGain        = CfgManager._Gains[i].gain.totalGain;
          gain[i].constantGain     = CfgManager._Gains[i].gain.constantGain;
          gain[i].rampGain         = CfgManager._Gains[i].gain.rampGain;
          gain[i].squareGain       = CfgManager._Gains[i].gain.squareGain;
          gain[i].sineGain         = CfgManager._Gains[i].gain.sineGain;
          gain[i].triangleGain     = CfgManager._Gains[i].gain.triangleGain;
          gain[i].sawtoothdownGain = CfgManager._Gains[i].gain.sawtoothdownGain;
          gain[i].sawtoothupGain   = CfgManager._Gains[i].gain.sawtoothupGain;
          gain[i].springGain       = CfgManager._Gains[i].gain.springGain;
          gain[i].damperGain       = CfgManager._Gains[i].gain.damperGain;
          gain[i].inertiaGain      = CfgManager._Gains[i].gain.inertiaGain;
          gain[i].frictionGain     = CfgManager._Gains[i].gain.frictionGain;
          gain[i].customGain       = CfgManager._Gains[i].gain.customGain;
}

          Joystick.setGains(gain);

}

/*
void calculateEncoderPostion_X() {
  encoder.tick_X();
}

void calculateEncoderPostion_Y() {
  encoder.tick_Y();
}
*/

void Push_Button_01_ISR()
{
  int bState = digitalReadFast(Buttons.pinNumber);
  if(Buttons.LastState != bState )
  {
     if((long)(millis() - Buttons.millis_time) > debouncing_time ) {

        Buttons.CurrentState = bState;
        Buttons.LastState = Buttons.CurrentState; 
        Buttons.millis_time = millis();
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
     
      encoder.axis[idx].currentPosition=0;
      encoder.updatePosition(idx);

}

void findCenter(int axis)
{
  char buff[48];
  int32_t LastPos=0, Axis_Center=0 ,Axis_Range=0;
  
  encoder.axis[axis].minValue =0;
  encoder.axis[axis].maxValue =0;
  Reset_Encoder(axis);
  pwm.servo_on(axis);
   delay(2000);
  //Serial.println("Find Center");
  while (Buttons.CurrentState)
  {
    encoder.updatePosition(axis);
    if(LastPos != encoder.axis[axis].currentPosition)
    {
    AutoCalibration(axis);
    LastPos = encoder.axis[axis].currentPosition;
    sprintf(buff,"[%d]: %ld,%ld", axis, encoder.axis[axis].minValue, encoder.axis[axis].maxValue);
    Serial.println(buff);
    }

  }
 
    Axis_Center= (encoder.axis[axis].minValue + encoder.axis[axis].maxValue)/2;
    Axis_Range =  abs(encoder.axis[axis].minValue) + abs(encoder.axis[axis].maxValue);
    encoder.axis[axis].maxValue = Axis_Range/2 -20;
    encoder.axis[axis].minValue = -encoder.axis[axis].maxValue;
    gotoPosition(axis, Axis_Center);    //goto center X
    Reset_Encoder(axis);
    sprintf(buff,"[%d]: %ld - 0 - %ld", axis, encoder.axis[axis].minValue, encoder.axis[axis].maxValue);
    Serial.println(buff);
    
    if(axis == X_AXIS)
    {
      Joystick.setXAxisRange(encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
      Joystick.setXAxis(encoder.axis[X_AXIS].currentPosition);
    }
    else
    {
       Joystick.setYAxisRange(encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
      Joystick.setYAxis(encoder.axis[Y_AXIS].currentPosition);
    }
    
    pwm.setPWM(axis, 0);
}


void gotoPosition(int idx, int32_t targetPosition) {
  int32_t LastPos=0;
  char buff[64];
  Setpoint[idx] = targetPosition;
  while (encoder.axis[idx].currentPosition != targetPosition) {
    Setpoint[idx] = targetPosition;
    encoder.updatePosition(idx);
    Input[idx] = encoder.axis[idx].currentPosition ;
    myPID[idx].Compute();
    pwm.setPWM(idx, -Output[idx]);
    CalculateMaxSpeedAndMaxAcceleration(idx);
    if (LastPos !=encoder.axis[idx].currentPosition )
    {
    sprintf(buff,"[%d] P: %ld,T: %ld,F: %d",idx,encoder.axis[idx].currentPosition, (int32_t)Setpoint[idx], (int)Output[idx] );
    Serial.println(buff);
    LastPos = encoder.axis[idx].currentPosition;
    }
    
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
