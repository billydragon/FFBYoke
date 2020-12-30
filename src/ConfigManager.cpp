#include <Arduino.h>
#include "ConfigManager.h"
#include "EEPROMAnything.h"

#ifdef _VARIANT_ARDUINO_DUE_X_
#define Serial  SerialUSB
#endif

DATA_TYPE data_type;
COMMAND_TYPE command_type;
COMMAND_HEADER CMD;
const GainsConfig default_Gains[]={{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
                                  {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100}};
const PidsConfig default_Pids[]={{255,1,2,0.5,0.01},{255,1,2,0.5,0.01}};

GainsConfig *uGains;   //={{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
                                //     {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100}};
PidsConfig *uPids;         //= {{255,1,2,0.5,0.01},{255,1,2,0.5,0.01}};
System_Control uSysControl;

template <class T> long int _writeAnything(T& valuebt)
{
long i = Serial.write((byte *) &valuebt, sizeof(valuebt));
  
  return i;
}


ConfigManager::ConfigManager( )
{

}

ConfigManager::~ConfigManager()
{
}


void ConfigManager::send_gains()
{
       
  for (int i = 0; i < 2; i++)
        {
          for (int j = 0; j < 13; j++)
          {
            CMD.Command = command_type.Read_Memory;
            CMD.Data_Type = data_type.Gains_Memory;
            CMD.Axis=i;
            CMD.Start_Index = j;
            CMD.Lenght = 1;
            _writeAnything(CMD.toBytes);
            _writeAnything(uGains[i].ToBytes[j]);   
            delay(20);
          }
          
        }
}

void ConfigManager::send_sys_control()
{
            byte cmd[3];
            cmd[0] = command_type.Read_Memory;
            cmd[1] = data_type.System_Memory;
            cmd[2] = uSysControl.ToByte;
            _writeAnything(cmd);
            delay(20);
      
}

void ConfigManager::send_Pids()
{
  for (int i = 0; i < 2; i++)
        {
          for (int j = 0; j < 5; j++)
          {
            CMD.Command = command_type.Read_Memory;
            CMD.Data_Type = data_type.Pids_Memory;
            CMD.Axis=i;
            CMD.Start_Index = j;
            CMD.Lenght = 4;
              _writeAnything(CMD.toBytes); 
              _writeAnything(uPids[i].ToBytes[j]);
              delay(20);
          }
        }
}

void ConfigManager::receive_Gains()
{
    delay(20);
    byte  idx = Serial.read();                  // 3 Axis index
    byte  pos = Serial.read();                  // 4 Position
    byte  cmdlen = Serial.read();         // 5 lenght of data
         
          for(int i = 0; i < (cmdlen); i++)
          {
                byte c = (byte)Serial.read();
                uGains[idx].ToBytes[pos + i] = c;

          }
}

void ConfigManager::receive_Pids()
{
    delay(20);
    byte idx = Serial.read();           // 3 Axis index
    byte pos = Serial.read();           // 4
    byte cmdlen = Serial.read();        // 5  lenght of data  
          
            for(int i = 0; i < (cmdlen/4); i++)
          {
            float_union f_union;
            f_union.uiBytes[0] = Serial.read();
            f_union.uiBytes[1] = Serial.read();
            f_union.uiBytes[2] = Serial.read();
            f_union.uiBytes[3] = Serial.read();

            uPids[idx].ToBytes[pos + i] = f_union.fValue;
                 
          }
}

void ConfigManager::receive_Sys_control()
{
      uSysControl.ToByte = Serial.read();

}


void ConfigManager::GetUpdate(GainsConfig *_gains, PidsConfig *_pids, System_Control _Sys_ctrl)
{
  if(_gains != nullptr){
		
	        uGains = _gains;
	      
	  }
  if(_pids != nullptr){

    uPids = _pids;
  
    }

  uSysControl = _Sys_ctrl;

byte cmd = 0;
byte dtype = 0;
  
  if(Serial.available()>0)
  {
    delay(20);
      cmd = Serial.read();
    if(cmd == command_type.Read_Memory )  //Read from Memory settings
    {
       dtype = Serial.read();
        switch (dtype)
          {
              case data_type.Gains_Memory:
                      send_gains();
                  
                break;
              case data_type.Pids_Memory: //request PID Config
                      send_Pids();
                 
              break;
              case data_type.System_Memory:  
                      send_sys_control();
              break;
              case data_type.All_Memory: //request all
                      send_gains();
                      send_Pids();
                      send_sys_control();
                     
              break;
              default:
                break;
            }
    }

    if (cmd == command_type.Write_Memory)  //Write to Memory settings         
    {
        dtype = Serial.read();    //2
          switch (dtype)
            {
                case data_type.Gains_Memory: //Send Gain Config   
                            receive_Gains();
                break;
                case data_type.Pids_Memory://Send PID Config    
                            receive_Pids();
                                                                      
                break;
                case data_type.System_Memory:  
                            receive_Sys_control();
                break;
                case data_type.All_Memory://Send PID Config  
                            receive_Gains();  
                            receive_Pids();
                            receive_Sys_control();
                break;
                default:
                break;
            }
            
    }

    if (cmd == command_type.Save_Eeprom)   // Save to EEPROM
    {
      
      dtype = Serial.read();    //2
          switch (dtype)
            {
              case data_type.Gains_Eeprom:
                     
                    EEPROM_writeAnything(ADDR_GAINS_START,uGains);
                    delay(50); 
              break;
              case data_type.Pids_Eeprom:
                    EEPROM_writeAnything(ADDR_PIDS_START,uPids);
                     delay(50);
              break;
              case data_type.System_Eeprom:
                    EEPROM_writeAnything(ADDR_START_SYSCONTROL,uSysControl.ToByte ); 
                    delay(50);
              break;
              case data_type.All_Eeprom:
                    EEPROM_writeAnything(ADDR_GAINS_START,uGains);
                    delay(50); 
                    EEPROM_writeAnything(ADDR_PIDS_START,uPids);
                    delay(50);
                    EEPROM_writeAnything(ADDR_START_SYSCONTROL,uSysControl.ToByte ); 
                    delay(50);

              break;
              default:
              break;
            }
    }

    if (cmd == command_type.Load_Eeprom)  //Load from EEPROM
    {
      
          dtype = Serial.read();
            switch(dtype)
            {
                    case data_type.Gains_Eeprom:
                          EEPROM_readAnything(ADDR_GAINS_START,uGains); 
                           delay(20);     
                          send_gains();
                    break;  
                     case data_type.Pids_Eeprom:
                          EEPROM_readAnything(ADDR_PIDS_START,uPids);
                           delay(20);  
                          send_Pids();
                    break; 
                    case data_type.System_Eeprom:
                          EEPROM_readAnything(ADDR_START_SYSCONTROL,uSysControl.ToByte ); 
                          send_sys_control(); 
                    case data_type.All_Eeprom:
                          EEPROM_readAnything(ADDR_GAINS_START,uGains);
                          delay(20);    
                          send_gains();
                          EEPROM_readAnything(ADDR_PIDS_START,uPids);
                           delay(20);  
                          send_Pids();
                          EEPROM_readAnything(ADDR_START_SYSCONTROL,uSysControl.ToByte ); 
                           delay(20); 
                          send_sys_control(); 
                    
                    break;
                    break;  
                   
                    default:
                    break;

            }
    }
    if (cmd == command_type.Control)  //Load from EEPROM
    {
                dtype = Serial.read();
                      switch(dtype)
                      {
                            case data_type.Reset_Default:
                                  memcpy(uGains, default_Gains,sizeof(default_Gains));
                                  delay(20);    
                                  send_gains();
                                  memcpy(uPids, default_Pids,sizeof(default_Gains));
                                  delay(20);  
                                  send_Pids();
                                  uSysControl.ToByte = 0;
                                  send_sys_control();
                                  
                            break; 
                            
                            default:
                            break;

                      }
    }
    
  
  }
  
}



