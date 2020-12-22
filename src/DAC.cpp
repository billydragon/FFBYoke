#include "DAC.h"
#include "YokeConfig.h"
#include "DigitalWriteFast.h"

DAC8562 dac8563;

DAC::DAC() 
{
        _cs = DEFAULT_CS_PIN;
        _vref = DEFAULT_VREF;
        
}

DAC::DAC(int cs) //ARDUINO BOARD MUST CS MUST BE 4,10 or 52
{
        _cs = cs;
        _vref = DEFAULT_VREF;
}
 
 DAC::~DAC() {
	 
 }

void DAC::begin()
{
        pinMode(SERVO_ON_X,OUTPUT);
	pinMode(SERVO_ON_Y,OUTPUT);
        delay(500); //wait for system startup.
        dac8563=DAC8562(_cs,_vref);

}

 void DAC::setPWM(int idx, int16_t force)
 {
        int16_t DACValue = map(force,-255,255,DAC_MAX,DAC_MIN); 
         switch (idx)
         {
         case X_AXIS:
                 dac8563.outPutValue(CMD_SETA_UPDATEA,DACValue);
                 break;
         case Y_AXIS:
                dac8563.outPutValue(CMD_SETB_UPDATEB,DACValue);
                break;
         default:
                 break;
         }
 }

 void DAC::servo_on(int idx)
{
	if(idx == 0)
	{ 
		digitalWriteFast(SERVO_ON_X,HIGH);
	}
	
	else
	digitalWriteFast(SERVO_ON_Y,HIGH);
}


void DAC::servo_off(int idx)
{
	if(idx == 0)
	{ 
		digitalWriteFast(SERVO_ON_X,LOW);
	}
	else
	digitalWriteFast(SERVO_ON_Y,LOW);
}



