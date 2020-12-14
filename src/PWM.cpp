#include "PWM.h"
#include "DigitalWriteFast.h"
 Pwm::Pwm(void) {

 }
 
 Pwm::~Pwm() {
	 
 }

void Pwm::begin(){
	pinMode(9,OUTPUT);
	pinMode(10,OUTPUT);
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
	setPWM_X(0);
    setPWM_Y(0);
  
}
 
void Pwm::setPWM_X(int16_t force) {
	//int nomalizedForce = map (force, -255,255, MINFORCE, MAXFORCE); 
	int nomalizedForce = map (force, -255,255,0,MAXFORCE);// MINFORCE, MAXFORCE); 
	PWM9 = nomalizedForce;
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
 
 void Pwm::setPWM_Y(int16_t force) {
	//int nomalizedForce = map (force, -255,255, MINFORCE, MAXFORCE); 
	int nomalizedForce = map (force, -255,255,0,MAXFORCE); // MINFORCE, MAXFORCE); 
	PWM10 = nomalizedForce;

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

 void Pwm::servo_on_X()
{
	digitalWriteFast(SERVO_ON_X,HIGH);
}

void Pwm::servo_on_Y()
{
	digitalWriteFast(SERVO_ON_Y,HIGH);
}

void Pwm::servo_off_X()
{
	digitalWriteFast(SERVO_ON_X,LOW);
}

void Pwm::servo_off_Y()
{
	digitalWriteFast(SERVO_ON_Y,LOW);
}

