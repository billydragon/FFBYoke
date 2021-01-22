/*
  DAC8563.cpp
  
  Arduino library for Texas Instruments DAC8563 2-channel 16-bit SPI DAC
  
  2018-07-24 @km7,  (cc) https://creativecommons.org/licenses/by/3.0/
*/

#include "DAC8563.h"
#include "DigitalWriteFast.h"

ConfigManager * _cfg_manager;

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

void DAC8563::begin(ConfigManager *cfg_mangager )
{

  _cfg_manager = cfg_mangager;
  pinMode(SERVO_ON_X,OUTPUT);
	pinMode(SERVO_ON_Y,OUTPUT);
  
  SPI.begin(_cs_pin);
  SPI.setDataMode(_cs_pin,SPI_MODE1);
  SPI.setBitOrder(_cs_pin,MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  initialize();
  
};

void DAC8563::setPWM(int axis, int32_t force)
 {
        uint16_t DACValue = 0;
        _Motor_Inv_X = _cfg_manager->_SysConfig.Byte.Motor_Inv_X;
        _Motor_Inv_Y = _cfg_manager->_SysConfig.Byte.Motor_Inv_Y;
       // _Motor_Dir_Delay = _cfg_manager->_SysConfig.Byte.Motor_Dir_Delay;
         switch (axis)
         {
            case X_AXIS:
                  if(_Motor_Inv_X == 1)
                  {
                      DACValue = map(force,-32767,32767,DAC_MAX,DAC_MIN);
                  }    
                  else
                  {
                      DACValue = map(force,-32767,32767,DAC_MIN,DAC_MAX); 
                  }
                    //  if(_Motor_Dir_Delay > 0)
                    //  {
                    //  outPutValue(CMD_SETA_UPDATEA, zeroPWM);
                    //  delay(_Motor_Dir_Delay);
                    //  }
                      WriteDAC(CMD_SET_A_UPDATE_A, DACValue);
                  break;
            case Y_AXIS:
                  if(_Motor_Inv_Y == 1)
                      DACValue = map(force,-32767,32767,DAC_MAX, DAC_MIN); 
                      else{            
                      DACValue = map(force,-32767,32767,DAC_MIN,DAC_MAX);
                      }
                    // if(_Motor_Dir_Delay > 0)
                    // {
                    //   outPutValue(CMD_SETB_UPDATEB, zeroPWM);
                    //   delay(_Motor_Dir_Delay);
                    // }
                      WriteDAC(CMD_SET_B_UPDATE_B, DACValue);
                  break;
            default:
                  break;
         }


 }

 

 void DAC8563::servo_on(int axis)
{
	if(axis == X_AXIS)
	{ 
		digitalWriteFast(SERVO_ON_X,LOW);
	}
	
	else
	digitalWriteFast(SERVO_ON_Y,LOW);
}


void DAC8563::servo_off(int axis)
{
	if(axis == X_AXIS)
	{ 
		digitalWriteFast(SERVO_ON_X,HIGH);
	}
	else
	  digitalWriteFast(SERVO_ON_Y,HIGH);
}


void DAC8563::DAC_WR_REG(uint8_t cmd_byte, uint16_t data_byte) {
  
  SPI.transfer(_cs_pin,cmd_byte,SPI_CONTINUE);
  SPI.transfer16(_cs_pin,data_byte);
 
};


void DAC8563::WriteDAC(uint8_t cmd_byte,uint16_t input) {
  byte inputMid = (input>>8)&0xFF;
  byte inputLast = input&0xFF;
  writeValue(cmd_byte,(inputMid),(inputLast));
  
};

void DAC8563::writeValue(uint8_t cmd_byte, uint8_t mid, uint8_t last) {
 
  SPI.transfer(_cs_pin,cmd_byte,SPI_CONTINUE);
  SPI.transfer(_cs_pin,mid,SPI_CONTINUE);
  SPI.transfer(_cs_pin,last,SPI_LAST);
  
};

void DAC8563::initialize() {
  DAC_WR_REG(CMD_RESET_REG, DATA_RESET_ALL_REG);      // reset
  DAC_WR_REG(CMD_PWR_UP_A_B, DATA_PWR_UP_A_B);        // power up
  DAC_WR_REG(CMD_INTERNAL_REF_EN, DATA_INTERNAL_REF_EN);      // enable internal reference
  DAC_WR_REG(CMD_GAIN, DATA_GAIN_B2_A2);            // set multiplier
  DAC_WR_REG(CMD_LDAC_DIS, DATA_LDAC_DIS_AB);          // update the caches
};

