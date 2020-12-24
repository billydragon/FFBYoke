/*
  DAC8563.cpp
  
  Arduino library for Texas Instruments DAC8563 2-channel 16-bit SPI DAC
  
  2018-07-24 @km7,  (cc) https://creativecommons.org/licenses/by/3.0/
*/

#include "DAC8563.h"
#include "DigitalWriteFast.h"

DAC8563::DAC8563()
{
  _cs_pin = CS_PIN;
  _vref = DEFAULT_VREF; //My Board using vref 3.3V
};

DAC8563::DAC8563(uint8_t cs_pin)
{
  _cs_pin = cs_pin;
  _vref= DEFAULT_VREF; //My Board using vref 3.3V
};

void DAC8563::begin()
{
  pinMode(SERVO_ON_X,OUTPUT);
	pinMode(SERVO_ON_Y,OUTPUT);

  /* !Chip select (low to enable) */
  pinMode(_cs_pin, OUTPUT);
  digitalWriteFast(_cs_pin, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setBitOrder(MSBFIRST);
  initialize();

  //Debug
 // SerialUSB.println("Init SPI Done.");
  //delay(1000);
};

void DAC8563::setPWM(int idx, int32_t force)
 {
        //char buff[64];
        uint16_t DACValue = map(force,-255,255,DAC856X_MIN,DAC856X_MAX); 
         switch (idx)
         {
         case X_AXIS:
                outPutValue(CMD_SETA_UPDATEA,DACValue);
                 break;
         case Y_AXIS:
                outPutValue(CMD_SETB_UPDATEB,DACValue);
                break;
         default:
                 break;
         }

        // sprintf(buff,"setPWM(%d): %d",idx, DACValue);
        // SerialUSB.println(buff);
 }

 void DAC8563::servo_on(int idx)
{
	if(idx == 0)
	{ 
		digitalWriteFast(SERVO_ON_X,HIGH);
	}
	
	else
	digitalWriteFast(SERVO_ON_Y,HIGH);
}


void DAC8563::servo_off(int idx)
{
	if(idx == 0)
	{ 
		digitalWriteFast(SERVO_ON_X,LOW);
	}
	else
	digitalWriteFast(SERVO_ON_Y,LOW);
}

void DAC8563::DAC_WR_REG(uint8_t cmd_byte, uint16_t data_byte) {
  
  digitalWriteFast(_cs_pin, LOW);
  SPI.transfer(cmd_byte);
  SPI.transfer16(data_byte);
  digitalWriteFast(_cs_pin, HIGH);
 
};

void DAC8563::outPutValue(uint8_t cmd_byte,uint16_t input) {
  byte inputMid = (input>>8)&0xFF;
  byte inputLast = input&0xFF;
  //unsigned int  t= (input>>8)&0xFF;
  writeValue(cmd_byte, (inputLast),(inputMid));
};


void DAC8563::writeValue(uint8_t cmd_byte, uint8_t mid, uint8_t last) {
 
 
  digitalWriteFast(_cs_pin, LOW);
  SPI.transfer(cmd_byte);
  SPI.transfer(last);
  SPI.transfer(mid);
  digitalWriteFast(_cs_pin, HIGH);
  
};

void DAC8563::initialize() {
  DAC_WR_REG(CMD_RESET_ALL_REG, DATA_RESET_ALL_REG);      // reset
  DAC_WR_REG(CMD_PWR_UP_A_B, DATA_PWR_UP_A_B);        // power up
  DAC_WR_REG(CMD_INTERNAL_REF_EN, DATA_INTERNAL_REF_EN);      // enable internal reference
  DAC_WR_REG(CMD_GAIN, DATA_GAIN_B2_A2);            // set multiplier
  DAC_WR_REG(CMD_LDAC_DIS, DATA_LDAC_DIS);          // update the caches
};

