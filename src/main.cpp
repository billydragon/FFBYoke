#include <Arduino.h>
#include "Joystick.h"
#include "DigitalWriteFast.h"
#include "YokeConfig.h"
#include "Encoder.h"
#include "PID_V2.h"
//#include "DAC8562.h"
#include "PWM.h"

#ifdef _VARIANT_ARDUINO_DUE_X_
#define Serial  SerialUSB
#endif


_Pwm pwm;
YokeConfig yokeConfig;
Encoder encoder; 
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

volatile int LimitSwitchState=1;
long debouncing_time = DEBOUNCE_TIME; //Debouncing Time in Milliseconds
volatile unsigned long LimitSwitch_last_micros=0;

bool initialRun = true;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
0, 0, // Button Count, Hat Switch Count
true, true, false, // X and Y, but no Z Axis
false, false, false, // No Rx, Ry, or Rz
false, false, // No rudder or throttle
false, false, false); // No accelerator, brake, or steering

Gains gain[2];
EffectParams effects[2];

void calculateEncoderPostion_X();
void calculateEncoderPostion_Y();
void YokeSetGains();
void GetForces();
void gotoPosition_X(int32_t targetPosition);
void CalculateMaxSpeedAndMaxAcceleration_X();
void gotoPosition_Y(int32_t targetPosition);
void CalculateMaxSpeedAndMaxAcceleration_Y();
void findCenter_X();
void findCenter_Y();
void LimitSwitch_ISR();

void setup() {
  // put your setup code here, to run once:
  pinMode(LIMIT_SWITCH,INPUT_PULLUP);
  encoder.setConfig(yokeConfig);
  attachInterrupt(digitalPinToInterrupt(encoderPin_XA), calculateEncoderPostion_X, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin_XB), calculateEncoderPostion_X, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(encoderPin_YA), calculateEncoderPostion_Y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin_YB), calculateEncoderPostion_Y, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH), LimitSwitch_ISR, FALLING);

  
  pwm.begin();
  pwm.setPWM_X(0);  
  pwm.setPWM_Y(0);
  pwm.servo_off_X();
  pwm.servo_off_Y();
  
  
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
  LimitSwitch_last_micros = millis();
}

void loop() {
  if (initialRun == true ) {
//    position control is not correctly, wheel runs over disired postion serveral times before stop
    
    findCenter_X();
    delay(1000);
    //findCenter_Y();
    initialRun = false;

  } else

  {
    // assign for re-test without initialRun
    //        Serial.print("currentVelocity: ");
    //        Serial.print(Wheel.encoder.maxVelocity);
    //        Serial.print(" maxAcceleration: ");
    //        Serial.println(Wheel.encoder.maxAcceleration);
    //        Serial.print("   maxPositionChange: ");
//    Serial.println(Wheel.encoder.currentPosition);
//    Wheel.encoder.maxPositionChange = 1151;
//    Wheel.encoder.maxVelocity  = 72;
//    Wheel.encoder.maxAcceleration = 33;
    encoder.updatePosition_X();

    if (encoder.axis[0].currentPosition > encoder.axis[0].maxValue) {
      Joystick.setXAxis(encoder.axis[0].maxValue);
    } else if (encoder.axis[0].currentPosition < encoder.axis[0].minValue) {
      Joystick.setXAxis(encoder.axis[0].minValue);
    } else {
      //Joystick.setXAxis(map(encoder.axis[0].currentPosition, encoder.axis[0].minValue , encoder.axis[0].maxValue, -32768, 32767));
    //gotoPosition_X( encoder.axis[0].currentPosition);
    Joystick.setXAxis(encoder.axis[0].currentPosition);
    }

    /*
    Serial.print("X: ");
    Serial.print(encoder.axis[0].currentPosition);
    Serial.print("/");
    Serial.print(map(encoder.axis[0].currentPosition, encoder.axis[0].minValue , encoder.axis[0].maxValue, -32768, 32767));
    */
    encoder.updatePosition_Y();
    if (encoder.axis[1].currentPosition > encoder.axis[1].maxValue) {
      Joystick.setYAxis(encoder.axis[1].maxValue);
    } else if (encoder.axis[1].currentPosition < encoder.axis[1].minValue) {
      Joystick.setYAxis(encoder.axis[1].minValue);
    } else {
      //Joystick.setYAxis(map(encoder.axis[1].currentPosition, encoder.axis[1].minValue , encoder.axis[1].maxValue, -32768, 32767));
        //gotoPosition_Y( encoder.axis[1].currentPosition);
        Joystick.setYAxis(encoder.axis[1].currentPosition);
    }


    /*
    Serial.print(" Y: ");
    Serial.print(encoder.axis[1].currentPosition);
    Serial.print("/");
    Serial.println(map(encoder.axis[1].currentPosition, encoder.axis[1].minValue , encoder.axis[1].maxValue, -32768, 32767));
    */
    GetForces();
  
    xy_force[0] = constrain(xy_force[0], -255, 255);
    xy_force[1] = constrain(xy_force[1], -255, 255);
    //  Serial.println(Wheel.encoder.currentPosition);
    //  when reach max and min wheel range, max force to prevent wheel goes over.
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
//  set total gain = 0.2 need replace by wheelConfig.totalGain.
  pwm.setPWM_X(xy_force[0] * TOTALGAIN_X);
  pwm.setPWM_Y(xy_force[1] * TOTALGAIN_Y);
    //Serial.print("Xf: ");
   // Serial.print(xy_force[0] * TOTALGAIN_X);
    //Serial.print(" Yf: ");
   // Serial.println((xy_force[1] * TOTALGAIN_Y));

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

void YokeSetGains(){

//set x axis gains
gain[0].totalGain = 100;
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
gain[1].totalGain = 100;
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

void calculateEncoderPostion_X() {
  encoder.tick_X();
}

void calculateEncoderPostion_Y() {
  encoder.tick_Y();
}

void LimitSwitch_ISR()
{
  if((long)(millis() - LimitSwitch_last_micros) >= debouncing_time ) {

   LimitSwitchState = LOW;
    
  LimitSwitch_last_micros = millis();
  }
      
}


void findCenter_X()
{
  char buff[48];
  int32_t LastPos=0;
  int32_t Xmin=0,Xmax=0,center_X=0;

  Serial.print("Finding X Axis Center");
    
  while (LimitSwitchState)
  {
    encoder.updatePosition_X();
    if(LastPos != encoder.axis[0].currentPosition)
    {
    sprintf(buff,"X_min: %ld",encoder.axis[0].currentPosition);
    Serial.println(buff);
    LastPos = encoder.axis[0].currentPosition;
    }
    
  }
        encoder.axis[0].currentPosition=0;
        Xmin = encoder.axis[0].currentPosition;      
  delay(500);
  LimitSwitchState= HIGH;
    
 while (LimitSwitchState)
  {
    encoder.updatePosition_X();
    if(LastPos != encoder.axis[0].currentPosition)
    {
    sprintf(buff,"X_max: %ld",encoder.axis[0].currentPosition);
    Serial.println(buff);
    LastPos = encoder.axis[0].currentPosition;
    }
  }
        Xmax = encoder.axis[0].currentPosition;
       
    center_X = (abs(Xmin) + Xmax)/2-2;
    encoder.axis[0].minValue = -center_X;
    encoder.axis[0].maxValue =  center_X;
    Joystick.setXAxisRange(encoder.axis[0].minValue,encoder.axis[0].maxValue);
    sprintf(buff,"Set X: %ld,0,%ld",encoder.axis[0].minValue, encoder.axis[0].maxValue);
    Serial.println(buff);
    pwm.servo_on_X();
    delay(2000);
    //encoder.updatePosition_X();
    gotoPosition_X(center_X);    //goto center X
    encoder.axis[0].currentPosition=0;
    Joystick.setXAxis(encoder.axis[0].currentPosition);
    delay(100);
    pwm.setPWM_X(0);
    LimitSwitchState= HIGH;
}


void findCenter_Y()
{
 
  char buff[48];
  int32_t LastPos =0;
  int32_t Ymin=0,Ymax=0,center_Y=0;

   Serial.print("Finding Y Axis Center");
   
  while (LimitSwitchState)
  {
    encoder.updatePosition_Y();
     if(LastPos != encoder.axis[1].currentPosition)
    {
    sprintf(buff,"Y_min: %ld",encoder.axis[1].currentPosition);
    Serial.println(buff);
      LastPos = encoder.axis[1].currentPosition;
    }
  }    
      encoder.axis[1].currentPosition=0;
      Ymin = encoder.axis[1].currentPosition;
      
 delay(500);
  LimitSwitchState= HIGH;
 while (LimitSwitchState)
  {
    encoder.updatePosition_Y();
     if(LastPos != encoder.axis[1].currentPosition)
    {
    sprintf(buff,"Y_max: %ld",encoder.axis[1].currentPosition);
    Serial.println(buff);
      LastPos = encoder.axis[1].currentPosition;
    }
  }
      Ymax = encoder.axis[1].currentPosition;
     
    center_Y = (abs(Ymin) + Ymax)/2-2;
    encoder.axis[1].minValue = -center_Y;
    encoder.axis[1].maxValue =  center_Y;
    Joystick.setYAxisRange(encoder.axis[1].minValue,encoder.axis[1].maxValue);
    sprintf(buff,"Set Y: %ld,0,%ld",encoder.axis[1].minValue, encoder.axis[1].maxValue);
    Serial.println(buff);
    pwm.servo_on_Y();
    delay(2000);
    //encoder.updatePosition_Y();
    gotoPosition_Y(center_Y);    //goto center Y
    encoder.axis[1].currentPosition=0;
    Joystick.setYAxis(encoder.axis[1].currentPosition);
    delay(100);
    pwm.setPWM_Y(0);
    LimitSwitchState= HIGH;
   
}


void gotoPosition_X(int32_t targetPosition) {
  char buff[64];
  Setpoint[0] = targetPosition;
  while (encoder.axis[0].currentPosition != targetPosition) {
    Setpoint[0] = targetPosition;
    encoder.updatePosition_X();
    Input[0] = encoder.axis[0].currentPosition ;
    myPID_X.Compute();
    pwm.setPWM_X(-Output[0]);
    CalculateMaxSpeedAndMaxAcceleration_X();
    sprintf(buff,"X Possition: %ld : Target: %ld : Force: %d",encoder.axis[0].currentPosition, (int32_t)Setpoint[0], (int)Output[0] );
    Serial.println(buff);
  }
  
}

void gotoPosition_Y(int32_t targetPosition) {
  char buff[64];
  Setpoint[1] = targetPosition;
  while (encoder.axis[1].currentPosition != targetPosition) {
    Setpoint[1] = targetPosition;
    encoder.updatePosition_Y();
    Input[1] = encoder.axis[1].currentPosition ;
    myPID_Y.Compute();
    pwm.setPWM_Y(-Output[1]);
    CalculateMaxSpeedAndMaxAcceleration_Y();
    sprintf(buff,"Y Possition: %ld : Target: %ld : Force: %d",encoder.axis[1].currentPosition, (int32_t)Setpoint[1], (int)Output[1] );
    Serial.println(buff);
  }
}

void CalculateMaxSpeedAndMaxAcceleration_X() {
  if (encoder.axis[0].maxVelocity < abs(encoder.axis[0].currentVelocity)) {
    encoder.axis[0].maxVelocity = abs(encoder.axis[0].currentVelocity);
  }
  if (encoder.axis[0].maxAcceleration < abs(encoder.axis[0].currentAcceleration)) {
    encoder.axis[0].maxAcceleration = abs(encoder.axis[0].currentAcceleration);
  }
  if (encoder.axis[0].maxPositionChange < abs(encoder.axis[0].positionChange)) {
    encoder.axis[0].maxPositionChange = abs(encoder.axis[0].positionChange);
  }
}

void CalculateMaxSpeedAndMaxAcceleration_Y( ) {
  if (encoder.axis[1].maxVelocity < abs(encoder.axis[1].currentVelocity)) {
    encoder.axis[1].maxVelocity = abs(encoder.axis[1].currentVelocity);
  }
  if (encoder.axis[1].maxAcceleration < abs(encoder.axis[1].currentAcceleration)) {
    encoder.axis[1].maxAcceleration = abs(encoder.axis[1].currentAcceleration);
  }
  if (encoder.axis[1].maxPositionChange < abs(encoder.axis[1].positionChange)) {
    encoder.axis[1].maxPositionChange = abs(encoder.axis[1].positionChange);
  }
}