#include <Arduino.h>
#include "Joystick.h"
#include "DigitalWriteFast.h"
#include "YokeConfig.h"
#include "Encoder.h"
#include "PWM_2.h"
#include "PID_V2.h"

Pwm pwm;
YokeConfig yokeConfig;
Encoder encoder; 
int32_t xy_force[2] = {0,0};
int32_t last_xy_force[2] = {0,0};

double Setpoint[2], Input[2], Output[2];
//double Kp=2, Ki=5, Kd=1;
double aggKp=4, aggKi=0.2, aggKd=1;
double Kp[2] = {0.01,0.01};
double Ki[2] = {0,0};
double Kd[2] = {0.05,0.5};
PID myPID_X(&Input[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT);
PID myPID_Y(&Input[1], &Output[1], &Setpoint[1], Kp[1], Ki[1], Kd[1], DIRECT);

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


void setup() {
  // put your setup code here, to run once:
  encoder.setConfig(yokeConfig);
  attachInterrupt(digitalPinToInterrupt(interrupt_XA), calculateEncoderPostion_X, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interrupt_XB), calculateEncoderPostion_X, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(interrupt_YA), calculateEncoderPostion_Y, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interrupt_YB), calculateEncoderPostion_Y, CHANGE);  
  pwm.begin();
  pwm.setPWM_X(0);  pwm.setPWM_Y(0);
  Input[0] = encoder.axis[0].currentPosition;
  Input[1] = encoder.axis[1].currentPosition;
  myPID_X.SetMode(AUTOMATIC);
  myPID_X.SetSampleTime(0.01);
  myPID_X.SetOutputLimits(-50, 50);
  myPID_Y.SetMode(AUTOMATIC);
  myPID_Y.SetSampleTime(0.01);
  myPID_Y.SetOutputLimits(-50, 50);
  Serial.begin(BAUD_RATE);

  Joystick.begin(true);
  YokeSetGains();

}

void loop() {
  if (initialRun == true ) {
//    position control is not correctly, wheel runs over disired postion serveral times before stop
    pwm.setPWM_X(10);
    gotoPosition_X(encoder.axis[0].minValue);
    gotoPosition_X(encoder.axis[0].maxValue);
    Joystick.setXAxisRange(encoder.axis[0].minValue,encoder.axis[0].maxValue);
    gotoPosition_X( 0);
    pwm.setPWM_X(0);
    delay(1000);
    pwm.setPWM_Y(10);
    gotoPosition_Y(encoder.axis[1].minValue);
    gotoPosition_Y(encoder.axis[1].maxValue);
    Joystick.setYAxisRange(encoder.axis[1].minValue,encoder.axis[1].maxValue);
    gotoPosition_Y( 0); 
    pwm.setPWM_X(0);
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
    encoder.updatePosition();

    if (encoder.axis[0].currentPosition > encoder.axis[0].maxValue) {
      Joystick.setXAxis(encoder.axis[0].maxValue);
    } else if (encoder.axis[0].currentPosition < encoder.axis[0].minValue) {
      Joystick.setXAxis(encoder.axis[0].minValue);
    } else {
      //Joystick.setXAxis(map(encoder.axis[0].currentPosition, encoder.axis[0].minValue , encoder.axis[0].maxValue, -32768, 32767));
    Joystick.setXAxis(encoder.axis[0].currentPosition);
    }

    /*
    Serial.print("X: ");
    Serial.print(encoder.axis[0].currentPosition);
    Serial.print("/");
    Serial.print(map(encoder.axis[0].currentPosition, encoder.axis[0].minValue , encoder.axis[0].maxValue, -32768, 32767));
    */

    if (encoder.axis[1].currentPosition > encoder.axis[1].maxValue) {
      Joystick.setYAxis(encoder.axis[1].maxValue);
    } else if (encoder.axis[1].currentPosition < encoder.axis[1].minValue) {
      Joystick.setYAxis(encoder.axis[1].minValue);
    } else {
      //Joystick.setYAxis(map(encoder.axis[1].currentPosition, encoder.axis[1].minValue , encoder.axis[1].maxValue, -32768, 32767));
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

}

void GetForces()
{
effects[0].springMaxPosition = encoder.axis[0].maxValue/2;
effects[1].springMaxPosition = encoder.axis[1].maxValue/2;
effects[0].frictionMaxPositionChange = encoder.axis[0].lastPosition - encoder.axis[0].currentPosition;
effects[1].frictionMaxPositionChange = encoder.axis[1].maxValue;
effects[0].inertiaMaxAcceleration = encoder.axis[0].maxValue;
effects[1].inertiaMaxAcceleration = 100;
effects[0].damperMaxVelocity = 10;
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

void gotoPosition_X(int32_t targetPosition) {
  Setpoint[0] = targetPosition;
  while (encoder.axis[0].currentPosition != targetPosition) {
    Setpoint[0] = targetPosition;
    encoder.updatePosition();
    Input[0] = encoder.axis[0].currentPosition ;
   
    myPID_X.Compute();
    pwm.setPWM_X(-Output[0]);
    CalculateMaxSpeedAndMaxAcceleration_X();
    Serial.print("X: ");
    Serial.print(encoder.axis[0].currentPosition);
    Serial.print(" : ");
    Serial.print(Setpoint[0]);
    Serial.print(" : ");
    //Serial.print("PWM: ");
    Serial.println(Output[0]);
  }
  
}

void gotoPosition_Y(int32_t targetPosition) {
  Setpoint[1] = targetPosition;
  while (encoder.axis[1].currentPosition != targetPosition) {
    Setpoint[1] = targetPosition;
    encoder.updatePosition();
    Input[1] = encoder.axis[1].currentPosition ;
   
    myPID_Y.Compute();
    pwm.setPWM_Y(-Output[1]);
    CalculateMaxSpeedAndMaxAcceleration_Y();
    Serial.print("Y: ");
    Serial.print(encoder.axis[1].currentPosition);
    Serial.print(" : ");
    Serial.print(Setpoint[1]);
    Serial.print(" : ");
    //Serial.print("PWM: ");
    Serial.println(Output[1]);
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