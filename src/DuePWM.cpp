/*  

Library:     pwm01.h (version 01)
Date:        2/11/2013
Written By:  randomvibe 

Library:	  DuePWM (version 1, based on pwm01.h version 01)
Date:		  6/17/2013
Rewritten By: exedor

Purpose:     
   Setup one or two unique PWM frequenices directly in sketch program,
   set PWM duty cycle, and stop PWM function.

User Functions:
   ctor: DuePWM();
   ctor: DuePWM(uint32_t clockA_freq, uint32_t clockB_freq)
   DuePWM::setFreq1( uint32_t clockA_freq ) // in Hz
   DuePWM::setFreq2( uint32_t clockB_freq ) // in Hz
   DuePWM::pinFreq1( uint32_t pin ) // pin must be 6 through 9
   DuePWM::pinFreq2( uint32_t pin ) // pin must be 6 through 9
   DuePWM::pinDuty( uint32_t pin_number, uint32_t duty_cycle) ~ Write PWM duty cycle 0 through 255
   DuePWM::stop( uint32_t pin_number) ~ Force PWM stop

Notes:
   - Applies to Arduino-Due board, PWM pins 6, 7, 8 & 9, all others ignored
   - Libary Does not operate on the TIO pins.
   - Unique frequencies set via PWM Clock-A ("CLKA") and Clock-B ("CLKB")
     Therefore, up to two unique frequencies allowed.
   - Set max duty cycle counts (pwm_max_duty_Ncount) equal to 255
     per Arduino approach.  This value is best SUITED for low frequency
     applications (2hz to 40,000hz) such as PWM motor drivers, 
     38khz infrared transmitters, etc.
   - Future library versions will address high frequency applications.
   - Arduino's "wiring_analog.c" function was very helpful in this effort.

 Rewrite Notes:
   - I hope the other isn't terribly upset with me for redoing his great original work :)
   
*/

#include "DuePWM.h"
#include "Arduino.h"
#include "DigitalWriteFast.h"


DuePWM::DuePWM()
{
	//pwm_res_nbit = 8;
	pmc_enable_periph_clk( PWM_INTERFACE_ID );
	setFreq1(PWM_FREQUENCY);
	setFreq2(PWM_FREQUENCY);
	set_output();
}


DuePWM::DuePWM(uint32_t clockA_freq, uint32_t clockB_freq)
{
	//pwm_res_nbit = res_nbit;
	pmc_enable_periph_clk( PWM_INTERFACE_ID );
	setFreq1(clockA_freq);
	setFreq2(clockB_freq);
	set_output();

}

void DuePWM::set_output()
{
	pinMode(Dir_X,OUTPUT);
    pinMode(Dir_Y,OUTPUT);
	pinMode(SERVO_ON_X,OUTPUT);
	pinMode(SERVO_ON_Y,OUTPUT);

}

void DuePWM::servo_on(int idx)
{
	if(idx == 0)
	{ 
		digitalWriteFast(SERVO_ON_X,HIGH);
	}
	
	else
	digitalWriteFast(SERVO_ON_Y,HIGH);
}


void DuePWM::servo_off(int idx)
{
	if(idx == 0)
	{ 
		digitalWriteFast(SERVO_ON_X,LOW);
	}
	else
	digitalWriteFast(SERVO_ON_Y,LOW);
}


//void pwm_set_resolution(int res)
//{
//	pwm_res_nbit = res;
//}

void DuePWM::setPWM(int idx, int16_t force) {
	int nomalizedForce=0;	
	int dir=0, pin=0;

	nomalizedForce = map (force, -255,255,0,DUE_PWM_SCALE); 
	if(idx == 0)
	{
		dir = Dir_X;
		pin = PWM_PIN_X;
	}
	else if (idx == 1)
	{
		dir = Dir_Y;
		pin = PWM_PIN_Y;
		/* code */
	}
		if (force >= 0) 
		{
			
			digitalWriteFast(dir,HIGH);
			pinDuty( pin, nomalizedForce); 
			
		}
		else
		{
			digitalWriteFast(dir,LOW);
			pinDuty(pin, nomalizedForce); 
		}
		
	
 }

void DuePWM::setFreq1(uint32_t  clockA_freq)
{
	pwm_clockA_freq = pwm_max_duty_Ncount*clockA_freq;

	// PWM STARTUP AND SETUP CLOCK
	//-------------------------------
	PWMC_ConfigureClocks( pwm_clockA_freq, pwm_clockB_freq, VARIANT_MCK );
}


void DuePWM::setFreq2(uint32_t  clockB_freq)
{
	pwm_clockB_freq = pwm_max_duty_Ncount*clockB_freq;
	
	// PWM STARTUP AND SETUP CLOCK
	//-------------------------------
	PWMC_ConfigureClocks( pwm_clockA_freq, pwm_clockB_freq, VARIANT_MCK );
}


// MAIN PWM INITIALIZATION
//--------------------------------
void  DuePWM::pinFreq1( uint32_t  pin )
{
    uint32_t  chan = g_APinDescription[pin].ulPWMChannel; 

	if (pin < 6 || pin > 9)
		return;
		
	// SET PWM RESOLUTION
	//------------------------
	//pwm_duty = mapResolution( pwm_duty, pwm_res_nbit, PWM_RESOLUTION);
 
	// SETUP PWM FOR pin
	//------------------------
	PIO_Configure( g_APinDescription[pin].pPort,  g_APinDescription[pin].ulPinType,
				   g_APinDescription[pin].ulPin,  g_APinDescription[pin].ulPinConfiguration);
	PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
	PWMC_SetPeriod(PWM_INTERFACE, chan, pwm_max_duty_Ncount);
	PWMC_SetDutyCycle(PWM_INTERFACE, chan, 0);  // The 0 is the initial duty cycle
	PWMC_EnableChannel(PWM_INTERFACE, chan);
}

void  DuePWM::pinFreq2( uint32_t  pin )
{
    uint32_t  chan = g_APinDescription[pin].ulPWMChannel; 

	if (pin < 6 || pin > 9)
		return;
		
	// SET PWM RESOLUTION
	//------------------------
	//pwm_duty = mapResolution( pwm_duty, pwm_res_nbit, PWM_RESOLUTION);
 
	// SETUP PWM FOR pin
	//------------------------
	PIO_Configure( g_APinDescription[pin].pPort,  g_APinDescription[pin].ulPinType,
				   g_APinDescription[pin].ulPin,  g_APinDescription[pin].ulPinConfiguration);
	PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKB, 0, 0);
	PWMC_SetPeriod(PWM_INTERFACE, chan, pwm_max_duty_Ncount);
	PWMC_SetDutyCycle(PWM_INTERFACE, chan, 0);  // The 0 is the initial duty cycle
	PWMC_EnableChannel(PWM_INTERFACE, chan);
}


// WRITE DUTY CYCLE
//--------------------------------
void  DuePWM::pinDuty( uint32_t  pin,  uint32_t  duty ) 
{
	if (pin < 6 || pin > 9)
		return;

	//pwm_duty = mapResolution( duty, pwm_res_nbit, PWM_RESOLUTION);
	PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[pin].ulPWMChannel, duty);
}



// FORCE PWM STOP
//--------------------------------
void  DuePWM::stop( uint32_t  pin ) 
{
    pinMode(pin, OUTPUT);      // sets the digital pin as output
    digitalWrite(pin, LOW);    // sets the LED off
}
