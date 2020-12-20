#include "PWM.h"
#include "DigitalWriteFast.h"
 _Pwm::_Pwm(void) {

 }
 
 _Pwm::~_Pwm() {
	 
 }

void _Pwm::begin(){

	pinMode(PWM_X,OUTPUT);
	pinMode(PWM_Y,OUTPUT);
	pinMode(Dir_X,OUTPUT);
    pinMode(Dir_Y,OUTPUT);
	pinMode(SERVO_ON_X,OUTPUT);
	pinMode(SERVO_ON_Y,OUTPUT);

    #ifdef _VARIANT_ARDUINO_DUE_X_
	analogWriteResolution(12);
	#else
	 // * PWM Pins 9 & 10, Timer 1 is using: Channel A OCR1A at Pin 9 and Channel B OCR1B  at Pin 10 *
	TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;   
    ICR1 = MAXFORCE;
    OCR1A = 0;
    OCR1B = 0;   
	#endif

	setPWM(X_AXIS,0);
    setPWM(Y_AXIS,0);

}
 
void _Pwm::setPWM(int idx, int16_t force) {
	int nomalizedForce=0;
	
	//int nomalizedForce = map (force, -255,255, MINFORCE, MAXFORCE); 

	#ifdef _VARIANT_ARDUINO_DUE_X_
	int dir=0, pin=0;
	nomalizedForce = map (force, -255,255,0,DAC_SCALE); 
	if(idx == 0)
	{
		dir = Dir_X;
		pin = PWM_X;
	}
	else if (idx == 1)
	{
		dir = Dir_Y;
		pin = PWM_Y;
		/* code */
	}
		if (force >= 0) 
		{
			
			digitalWriteFast(dir,HIGH);
			analogWrite(pin,nomalizedForce);
			
		}
		else
		{
			digitalWriteFast(dir,LOW);
			analogWrite(pin,abs(nomalizedForce));
		}
		
	#else
	nomalizedForce = map (force, -255,255,0,MAXFORCE);
	if(idx == X_AXIS)
	{
		OCR1A = abs(nomalizedForce);
	}	
	else if(idx == Y_AXIS)
	{
		OCR1B = abs(nomalizedForce);
	}
		
		
	#endif
	
 }
 
 void _Pwm::servo_on(int idx)
{
	if(idx == X_AXIS)
	{
		digitalWriteFast(SERVO_ON_X,HIGH);
	}	
	else if(idx == Y_AXIS)
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
	else if(idx == Y_AXIS)
	{
		digitalWriteFast(SERVO_ON_Y,LOW);
	}
		
}

