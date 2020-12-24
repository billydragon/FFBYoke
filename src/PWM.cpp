#include "PWM.h"
#include "DigitalWriteFast.h"
 _Pwm::_Pwm(void) {

 }
 
 _Pwm::~_Pwm() {
	 
 }

void _Pwm::begin(){

	pinMode(PWM_X,OUTPUT);
	pinMode(PWM_Y,OUTPUT);
	//pinMode(Dir_X,OUTPUT);
    //pinMode(Dir_Y,OUTPUT);
	pinMode(SERVO_ON_X,OUTPUT);
	pinMode(SERVO_ON_Y,OUTPUT);

	 // * PWM Pins 9 & 10, Timer 1 is using: Channel A OCR1A at Pin 9 and Channel B OCR1B  at Pin 10 *
	TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;   
    ICR1 = MAXFORCE;
    OCR1A = 0;
    OCR1B = 0;   
	
	setPWM(X_AXIS,0);
    setPWM(Y_AXIS,0);

}
 
void _Pwm::setPWM(int idx, int16_t force) {
	int nomalizedForce=0;
	
	//int nomalizedForce = map (force, -255,255, MINFORCE, MAXFORCE); 

	nomalizedForce = map (force, -255,255,0,MAXFORCE);
	if(idx == 0)
	{
		
		OCR1A = abs(nomalizedForce);
	}
	else if (idx == 1)
	{
		OCR1B = abs(nomalizedForce);
		
	}
	
	
 }
 
 void _Pwm::servo_on(int idx)
{
	if(idx == X_AXIS)
	{ 
		digitalWriteFast(SERVO_ON_X,HIGH);
	}
	else
	{
		digitalWriteFast(SERVO_ON_Y,HIGH);
	}
	
}


void _Pwm::servo_off(int idx)
{
	if(idx == X_AXIS)
	{ 
		digitalWriteFast(SERVO_ON_X,LOW);
	}
	else
	{
		digitalWriteFast(SERVO_ON_Y,LOW);
	}
	
}

