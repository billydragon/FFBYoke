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

/*
DAC8563::DAC8563( uint8_t cs_pin, float vref)
{
  _cs_pin = cs_pin;
  _vref=vref;
};
*/
void DAC8563::begin(ConfigManager *cfg_mangager )
{

  _cfg_manager = cfg_mangager;
  pinMode(SERVO_ON_X,OUTPUT);
	pinMode(SERVO_ON_Y,OUTPUT);
  //pinMode(LDAC_PIN,OUTPUT);

  SPI.begin(_cs_pin);
  SPI.setDataMode(_cs_pin,SPI_MODE1);
  SPI.setBitOrder(_cs_pin,MSBFIRST);
  //SPI.setClockDivider(SPI_CLOCK_DIV8);
  initialize();
  
};

void DAC8563::setPWM(int idx, int32_t force)
 {
        uint16_t DACValue = 0;
        //uint16_t zeroPWM = map(0,-255,255,DAC_MIN,DAC_MAX);
        _Motor_Inv_X = _cfg_manager->_SysConfig.Byte.Motor_Inv_X;
        _Motor_Inv_Y = _cfg_manager->_SysConfig.Byte.Motor_Inv_Y;
       // _Motor_Dir_Delay = _cfg_manager->_SysConfig.Byte.Motor_Dir_Delay;
         switch (idx)
         {
         case X_AXIS:
                if(_Motor_Inv_X == 1)
                    DACValue = map(force,-255,255,DAC_MAX,DAC_MIN);
                else{
                    DACValue = map(force,-255,255,DAC_MIN,DAC_MAX); 
                    }
                  //  if(_Motor_Dir_Delay > 0)
                  //  {
                  //  outPutValue(CMD_SETA_UPDATEA, zeroPWM);
                  //  delay(_Motor_Dir_Delay);
                  //  }
                    //outPutValue(CMD_SETB_UPDATEB, zeroPWM);
                    outPutValue(CMD_SET_A_UPDATE_A, DACValue);
                 break;
         case Y_AXIS:
                 if(_Motor_Inv_Y == 1)
                    DACValue = map(force,-255,255,DAC_MAX, DAC_MIN); 
                    else{            
                    DACValue = map(force,-255,255,DAC_MIN,DAC_MAX);
                    }
                   // if(_Motor_Dir_Delay > 0)
                   // {
                   //   outPutValue(CMD_SETB_UPDATEB, zeroPWM);
                   //   delay(_Motor_Dir_Delay);
                   // }
                   // outPutValue(CMD_SETA_UPDATEA, zeroPWM);
                    outPutValue(CMD_SET_B_UPDATE_B, DACValue);
                break;
         default:
                 break;
         }


 }

 void DAC8563::servo_on(int idx)
{
	if(idx == X_AXIS)
	{ 
		digitalWriteFast(SERVO_ON_X,LOW);
	}
	
	else
	digitalWriteFast(SERVO_ON_Y,LOW);
}


void DAC8563::servo_off(int idx)
{
	if(idx == X_AXIS)
	{ 
		digitalWriteFast(SERVO_ON_X,HIGH);
	}
	else
	  digitalWriteFast(SERVO_ON_Y,HIGH);
}


void DAC8563::DAC_WR_REG(uint8_t cmd_byte, uint16_t data_byte) {
  
  SPI.transfer(_cs_pin,cmd_byte);
  SPI.transfer16(_cs_pin,data_byte);
 
};


void DAC8563::outPutValue(uint8_t cmd_byte,uint16_t input) {
  //byte inputMid = (input>>8)&0xFF;
  //byte inputLast = input&0xFF;
  //writeValue(cmd_byte, (inputLast),(inputMid));
  write(cmd_byte, input);
};

void DAC8563::writeVoltage(float input) {
  writeA(input);
  writeB(input);
};

void DAC8563::writeA(float input) {
 outPutValue(CMD_SET_A_UPDATE_A,Voltage_Convert(input/_vref*5));
};

void DAC8563::writeB(float input) {
 outPutValue(CMD_SET_B_UPDATE_B,Voltage_Convert(input/_vref*5));
};

void DAC8563::writeValue(uint8_t cmd_byte, uint8_t mid, uint8_t last) {
  //digitalWriteFast(LDAC_PIN,HIGH);
  SPI.transfer(_cs_pin,cmd_byte);
  SPI.transfer(_cs_pin,last);
  SPI.transfer(_cs_pin,mid);
  //digitalWriteFast(LDAC_PIN,LOW);

};

void DAC8563::write(uint8_t cmd_byte, uint16_t data)
{ 
  //uint8_t datahigh; 
  uint8_t datamid, datalow;

  //datahigh = (uint8_t) ((data >> 16) & 0xFF); 
  datamid  = (uint8_t) ((data >>  8) & 0xFF);
  datalow  = (uint8_t) ((data >>  0) & 0xFF);
    SPI.transfer(cmd_byte,SPI_CONTINUE);
    SPI.transfer(datamid,SPI_CONTINUE);
    SPI.transfer(datalow, SPI_LAST);
}

void DAC8563::initialize() {
  DAC_WR_REG(CMD_RESET_REG, DATA_RESET_ALL_REG);      // reset
  DAC_WR_REG(CMD_PWR_UP_A_B, DATA_PWR_UP_A_B);        // power up
  DAC_WR_REG(CMD_INTERNAL_REF_EN, DATA_INTERNAL_REF_EN);      // enable internal reference
  DAC_WR_REG(CMD_GAIN, DATA_GAIN_B2_A2);            // set multiplier
  DAC_WR_REG(CMD_LDAC_DIS, DATA_LDAC_DIS_AB);          // update the caches
};

uint16_t DAC8563::Voltage_Convert(float voltage)
{
  uint16_t _D_;
  
  voltage = voltage / 6  + 2.5;   //based on the manual provided by texas instruments

  _D_ = (uint16_t)(65536 * voltage / 5);

  if(_D_ < 32768)
  {
    _D_ -= 100;     //fix the errors
  }
    
  return _D_;
};
