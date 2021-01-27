#include <Arduino.h>
#include "Joystick.h"
#include "DigitalWriteFast.h"
#include "YokeConfig.h"
#include "ConfigManager.h"
#include "PID_V2.h"



#define DEBUG

#define Serial  SerialUSB

#include "DAC8563.h"
DAC8563 pwm=DAC8563(CS_PIN);

#include "Due_QDEC.h"
Due_QDEC encoder;


int32_t xy_force[2] = {0,0};


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
NUM_OF_BUTTONS, NUM_OF_HATSWITCH, // Button Count, Hat Switch Count
true, true, true, // X and Y, but no Z Axis
true, true, true, // No Rx, Ry, or Rz
true, true, // No rudder or throttle
true, true, true); // No accelerator, brake, or steering

BUTTONS Buttons[NUM_OF_BUTTONS]; 

volatile uint8_t initialRun = 0;

Gains gain[2];
EffectParams effects[2];

ConfigManager CfgManager = ConfigManager();
YokeConfig yokeConfig;
void calculateEncoderPostion(int idx);
void Set_Gains(void);
void SetEffects(void);
void gotoPosition(int idx, int32_t targetPosition);
void CalculateMaxSpeedAndMaxAcceleration(int idx);
void findCenter(int axis);
void Push_Button_01_ISR(void);
void Update_Joystick_Buttons(void);
void Set_PID_Turnings(void);

void Set_PID_Turnings()
{

  for(int ax =0; ax <2 ; ax++)
  {
  Kp[ax]= CfgManager._Pids[ax].Pid.Kp;
  Ki[ax] = CfgManager._Pids[ax].Pid.Ki;
  Kd[ax] = CfgManager._Pids[ax].Pid.Kd;
  myPID[ax].SetTunings(Kp[ax],Ki[ax],Kd[ax]);
  myPID[ax].SetSampleTime(CfgManager._Pids[ax].Pid.SampleTime);
  myPID[ax].SetOutputLimits(-CfgManager._Pids[ax].Pid.MaxOutput, CfgManager._Pids[ax].Pid.MaxOutput);
  }
 
}

void setup() {
 
  delay(1500);
  Serial.begin(BAUD_RATE);
  Joystick.begin(true);
  CfgManager.begin();  
   for(int ax =0; ax <2 ; ax++)
  {
  
  myPID[ax].SetMode(AUTOMATIC);
  Input[ax] = encoder.axis[ax].currentPosition;
  }
  Set_PID_Turnings();
  Buttons[0].pinNumber = PUSH_BUTTON_01;
  Buttons[0].CurrentState = HIGH;
  Buttons[0].LastState = HIGH;
  Buttons[0].millis_time = millis();

  pinMode(Buttons[0].pinNumber,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Buttons[0].pinNumber), Push_Button_01_ISR, CHANGE);

  pwm.begin(&CfgManager);
  pwm.servo_on(X_AXIS);
  pwm.servo_on(Y_AXIS);
  pwm.setPWM(X_AXIS,0);  
  pwm.setPWM(Y_AXIS,0);
  
  initialRun = CfgManager._SysConfig.Byte.Auto_Calibration;
  
}

void loop() {

  CfgManager.GetUpdate();
  Set_PID_Turnings(); 
  
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
    
    xy_force[X_AXIS] = constrain(xy_force[X_AXIS], -32767, 32767);
    xy_force[Y_AXIS] = constrain(xy_force[Y_AXIS], -32767, 32767);
    
    if (encoder.axis[X_AXIS].currentPosition >= encoder.axis[X_AXIS].maxValue) {
      xy_force[X_AXIS] = 32767;
    } else if (encoder.axis[X_AXIS].currentPosition <= encoder.axis[X_AXIS].minValue) {
      xy_force[X_AXIS] = -32767;
    }
    if (encoder.axis[Y_AXIS].currentPosition >= encoder.axis[Y_AXIS].maxValue) {
      xy_force[Y_AXIS] = 32767;
    } else if (encoder.axis[Y_AXIS].currentPosition <= encoder.axis[Y_AXIS].minValue) {
      xy_force[Y_AXIS] = -32767;
    }

    pwm.setPWM(X_AXIS,xy_force[X_AXIS]);
    CalculateMaxSpeedAndMaxAcceleration(X_AXIS);
    pwm.setPWM(Y_AXIS,xy_force[Y_AXIS]);
    CalculateMaxSpeedAndMaxAcceleration(Y_AXIS);
    Update_Joystick_Buttons();
  
}


void Update_Joystick_Buttons()
{

  for (int i = 0; i < NUM_OF_BUTTONS; i++) {
		if (Buttons[i].LastState != Buttons[i].CurrentState) {
			Buttons[i].LastState = Buttons[i].CurrentState;
			Joystick.setButton(i, Buttons[i].CurrentState);
		}
	}
    
     
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
          gain[i].totalGain        = CfgManager._Gains[i].Gain.totalGain;
          gain[i].constantGain     = CfgManager._Gains[i].Gain.constantGain;
          gain[i].rampGain         = CfgManager._Gains[i].Gain.rampGain;
          gain[i].squareGain       = CfgManager._Gains[i].Gain.squareGain;
          gain[i].sineGain         = CfgManager._Gains[i].Gain.sineGain;
          gain[i].triangleGain     = CfgManager._Gains[i].Gain.triangleGain;
          gain[i].sawtoothdownGain = CfgManager._Gains[i].Gain.sawtoothdownGain;
          gain[i].sawtoothupGain   = CfgManager._Gains[i].Gain.sawtoothupGain;
          gain[i].springGain       = CfgManager._Gains[i].Gain.springGain;
          gain[i].damperGain       = CfgManager._Gains[i].Gain.damperGain;
          gain[i].inertiaGain      = CfgManager._Gains[i].Gain.inertiaGain;
          gain[i].frictionGain     = CfgManager._Gains[i].Gain.frictionGain;
          gain[i].customGain       = CfgManager._Gains[i].Gain.customGain;
      }

          Joystick.setGains(gain);

}

void Push_Button_01_ISR()
{
  int bState = digitalReadFast(Buttons[0].pinNumber);
  if(Buttons[0].CurrentState != bState )
  {
     if((long)(millis() - Buttons[0].millis_time) > debouncing_time ) {

        Buttons[0].CurrentState = bState;
        Buttons[0].millis_time = millis();
    }
    else
    {
       Buttons[0].CurrentState = digitalReadFast(Buttons[0].pinNumber);
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

void SetZero_Encoder(int idx)
{
     
      encoder.axis[idx].currentPosition=0;
      encoder.Reset_Encoder(idx);
      encoder.updatePosition(idx);

}

void findCenter(int axis_num)
{
  char buff[48];
  int32_t LastPos=0, Axis_Center=0 ,Axis_Range=0;
  
  encoder.axis[axis_num].minValue =0;
  encoder.axis[axis_num].maxValue =0;
  SetZero_Encoder(axis_num);
  
  while (Buttons[0].CurrentState)
  {
    encoder.updatePosition(axis_num);
    if(LastPos != encoder.axis[axis_num].currentPosition)
    {
    AutoCalibration(axis_num);
    LastPos = encoder.axis[axis_num].currentPosition;
    #ifdef DEBUG
    sprintf(buff,"[%d]: %ld,%ld", axis_num, encoder.axis[axis_num].minValue, encoder.axis[axis_num].maxValue);
    Serial.println(buff);
    #endif
    }

  }
 
    Axis_Center= (encoder.axis[axis_num].minValue + encoder.axis[axis_num].maxValue)/2;
    Axis_Range =  abs(encoder.axis[axis_num].minValue) + abs(encoder.axis[axis_num].maxValue);
   
    #ifdef DEBUG
    sprintf(buff,"[%d]: %ld - 0 - %ld", axis_num, encoder.axis[axis_num].minValue, encoder.axis[axis_num].maxValue);
    Serial.println(buff);
    #endif
    

    switch (axis_num)
    {
    case X_AXIS:
      gotoPosition(X_AXIS, Axis_Center);    //goto center X
      SetZero_Encoder(X_AXIS);
      encoder.axis[X_AXIS].maxValue = (Axis_Range - AXIS_EDGE_PROTECT)/2;
    encoder.axis[X_AXIS].minValue = -encoder.axis[X_AXIS].maxValue;
      Joystick.setXAxisRange(encoder.axis[X_AXIS].minValue, encoder.axis[X_AXIS].maxValue);
      Joystick.setXAxis(encoder.axis[X_AXIS].currentPosition);
       pwm.setPWM(X_AXIS, 0);
      break;
    case Y_AXIS:
      gotoPosition(Y_AXIS, Axis_Center);    //goto center X
      SetZero_Encoder(Y_AXIS);
      encoder.axis[Y_AXIS].maxValue = (Axis_Range - AXIS_EDGE_PROTECT)/2;
      encoder.axis[Y_AXIS].minValue = -encoder.axis[Y_AXIS].maxValue;
      Joystick.setYAxisRange(encoder.axis[Y_AXIS].minValue, encoder.axis[Y_AXIS].maxValue);
      Joystick.setYAxis(encoder.axis[Y_AXIS].currentPosition);
       pwm.setPWM(Y_AXIS, 0);
      break;
    default:
      break;
    }
   
   
}


void gotoPosition(int axis_num, int32_t targetPosition) {
  int32_t LastPos=0;
  
  Setpoint[axis_num] = targetPosition;
  while (encoder.axis[axis_num].currentPosition != targetPosition) {
    Setpoint[axis_num] = targetPosition;
    encoder.updatePosition(axis_num);
    Input[axis_num] = encoder.axis[axis_num].currentPosition ;
    myPID[axis_num].Compute();
    pwm.setPWM(axis_num, -Output[axis_num]);
    CalculateMaxSpeedAndMaxAcceleration(axis_num);
    
    #ifdef DEBUG
    if (LastPos !=encoder.axis[axis_num].currentPosition )
    {
    char buff[64];
    sprintf(buff,"[%d] P: %ld,T: %ld,F: %d",axis_num,encoder.axis[axis_num].currentPosition, (int32_t)Setpoint[axis_num], (int)Output[axis_num]);
    Serial.println(buff);
    
    LastPos = encoder.axis[axis_num].currentPosition;
    }
    #endif
    
  }
  
}

void CalculateMaxSpeedAndMaxAcceleration(int axis_num) {
  if (encoder.axis[axis_num].maxVelocity < abs(encoder.axis[axis_num].currentVelocity)) {
    encoder.axis[axis_num].maxVelocity = abs(encoder.axis[axis_num].currentVelocity);
  }
  if (encoder.axis[axis_num].maxAcceleration < abs(encoder.axis[axis_num].currentAcceleration)) {
    encoder.axis[axis_num].maxAcceleration = abs(encoder.axis[axis_num].currentAcceleration);
  }
  if (encoder.axis[axis_num].maxPositionChange < abs(encoder.axis[axis_num].positionChange)) {
    encoder.axis[axis_num].maxPositionChange = abs(encoder.axis[axis_num].positionChange);
  }
}
