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

	setPWM_X(0);
    setPWM_Y(0);

  
}
 
void _Pwm::setPWM_X(int16_t force) {
	//int nomalizedForce = map (force, -255,255, MINFORCE, MAXFORCE); 
	#ifdef _VARIANT_ARDUINO_DUE_X_
	int nomalizedForce = map (force, -255,255,0,DAC_SCALE); 
		if (force >= 0) 
		{
			digitalWriteFast(Dir_X,HIGH);
			
		}
		else
		{
			digitalWriteFast(Dir_X,LOW);
		}
		analogWrite(PWM_X,nomalizedForce);
	#else
		int nomalizedForce = map (force, -255,255,0,MAXFORCE);// MINFORCE, MAXFORCE); 
		PWM_X = nomalizedForce;
	#endif
	/*
	if (nomalizedForce >= 0) {
		digitalWriteFast(Dir_X,HIGH);
		PWM9 = nomalizedForce;
		//analogWrite(9,abs(nomalizedForce));
		//PWM9 = 0;
	} else {
		//PWM10 = 0;
		digitalWriteFast(Dir_X,LOW);
		//analogWrite(9,abs(nomalizedForce));
		PWM9 = abs(nomalizedForce);
	}
	*/
 }
 
 void _Pwm::setPWM_Y(int16_t force) {
	//int nomalizedForce = map (force, -255,255, MINFORCE, MAXFORCE); 
	#ifdef _VARIANT_ARDUINO_DUE_X_
	int nomalizedForce = map (force, -255,255,0,DAC_SCALE); 
		if (force >= 0) 
		{
			digitalWriteFast(Dir_Y,HIGH);
			
		}
		else
		{
			digitalWriteFast(Dir_Y,LOW);
		}
		analogWrite(PWM_Y,nomalizedForce);
	#else
	int nomalizedForce = map (force, -255,255,0,MAXFORCE); // MINFORCE, MAXFORCE); 
	PWM_Y = nomalizedForce;
	#endif

	/*
	if (nomalizedForce >= 0) {
		digitalWriteFast(Dir_Y,HIGH);
		//analogWrite(10,abs(nomalizedForce));
		PWM10 = nomalizedForce;
		//PWM9 = 0;
	} else {
		//PWM10 = 0;
		digitalWriteFast(Dir_Y,LOW);
		//analogWrite(10,abs(nomalizedForce));
		PWM10 = abs(nomalizedForce);
	}
	*/
 }

 void _Pwm::servo_on_X()
{
	digitalWriteFast(SERVO_ON_X,HIGH);
}

void _Pwm::servo_on_Y()
{
	digitalWriteFast(SERVO_ON_Y,HIGH);
}

void _Pwm::servo_off_X()
{
	digitalWriteFast(SERVO_ON_X,LOW);
}

void _Pwm::servo_off_Y()
{
	digitalWriteFast(SERVO_ON_Y,LOW);
}

