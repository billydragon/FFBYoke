#include <Arduino.h>
#include "EEPROMAnything.h"
#include "ConfigManager.h"


#ifdef _VARIANT_ARDUINO_DUE_X_
#define Serial  SerialUSB
#endif

DATA_TYPE data_type;
COMMAND_TYPE command_type;
COMMAND_HEADER CMD;

const GainsConfig default_Gains[]={GainsConfig{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
                                  GainsConfig{100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100}};
const PidsConfig default_Pids[]={PidsConfig{255,1,2,0.5,0.01},PidsConfig{255,1,2,0.5,0.01}};
const System_Config default_SysConfig = System_Config{0,0,0,0,0,0,0,0};



template <class T> long int _writeAnything(T& valuebt)
{
long i = Serial.write((byte *) &valuebt, sizeof(valuebt));
  
  return i;
}


ConfigManager::ConfigManager()
{

  EEPROM_readAnything(ADDR_RESET_FLAG,Reset_Flag);
  if(Reset_Flag != 1)
  {
    Reset_to_Default();
    Write_All_EEPROM();
    first_run = 0;
  }
  else
  {
    first_run = 1;
  }
  
      
      
}

void ConfigManager::begin() 
{
      
      if(first_run == 1)
      {
        Read_All_EEPROM();
        first_run = 0;
      }
      
}

ConfigManager::~ConfigManager()
{
}

void ConfigManager::send_Gains(byte dt)
{
  for (int i = 0; i < 2; i++)
        {
          for (int j = 0; j < 13; j++)
          {
            CMD.Command = command_type.Read_Memory;
            CMD.Data_Type = dt;
            CMD.Axis=i;
            CMD.Start_Index = j;
            CMD.Lenght = 1;
            _writeAnything(CMD.toBytes);
            _writeAnything(_Gains[i].ToArray[j]);   
            //delay(20);
          }
          
        }
}


void ConfigManager::send_Pids(byte dt)
{
  for (int i = 0; i < 2; i++)
        {
          for (int j = 0; j < 5; j++)
          {
            CMD.Command = command_type.Read_Memory;
            CMD.Data_Type = dt;
            CMD.Axis=i;
            CMD.Start_Index = j;
            CMD.Lenght = sizeof(float);
              _writeAnything(CMD.toBytes); 
              _writeAnything(_Pids[i].ToArray[j]);
              //delay(20);
          }
        }
}


void ConfigManager::send_SysConfig(byte dt)
{
           _SysConfig.Byte_Flags = Flags.ToByte;
          for(int j = 0 ; j < 8 ; j++)
          {
            //byte cmd[3];
            CMD.Command = command_type.Read_Memory;
            CMD.Data_Type = dt;
            CMD.Axis = 0;
            CMD.Start_Index = j;
            CMD.Lenght  = 1;
            _writeAnything(CMD.toBytes);
            _writeAnything(_SysConfig.ToArray[j]);
            
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
                _Gains[idx].ToArray[pos + i] = c;

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

            _Pids[idx].ToArray[pos + i] = f_union.fValue;
                 
          }
}



void ConfigManager::receive_SysConfig()
{     
      delay(20);
      byte idx = Serial.read();           // 3 Axis index
      byte pos = Serial.read();           // 4
      byte cmdlen = Serial.read();        // 5  lenght of data  
     
        for(int j=0; j < cmdlen ; j++)
        {  
            byte c = (byte)Serial.read();
          _SysConfig.ToArray[pos + j] = c;
        }
      Flags.ToByte = _SysConfig.Byte_Flags;

}

void ConfigManager::Read_All_EEPROM()
{
        Read_Gains_EEPROM();
        Read_Pids_EEPROM();
        Read_SysConfig_EEPROM();
}

void ConfigManager::Read_Gains_EEPROM()
{
      EEPROM_readAnything(ADDR_GAINS_START,_Gains);
}

void ConfigManager::Read_Pids_EEPROM()
{
     EEPROM_readAnything(ADDR_PIDS_START,_Pids);
}

void ConfigManager::Read_SysConfig_EEPROM()
{
     EEPROM_readAnything(ADDR_START_SYSCONFIG,_SysConfig.ToArray); 
        Flags.ToByte = _SysConfig.Byte_Flags;
}

void ConfigManager::Write_Gains_EEPROM()
{
    EEPROM_writeAnything(ADDR_GAINS_START,_Gains);
        delay(10); 
}

void ConfigManager::Write_Pids_EEPROM()
{
    EEPROM_writeAnything(ADDR_PIDS_START,_Pids);
        delay(10);
}

void ConfigManager::Write_SysConfig_EEPROM()
{
    _SysConfig.Byte_Flags = Flags.ToByte;
    EEPROM_writeAnything(ADDR_START_SYSCONFIG,_SysConfig.ToArray ); 
    delay(10);
}

void ConfigManager::Write_All_EEPROM()
{
        Write_Gains_EEPROM();
        Write_Pids_EEPROM();
        Write_SysConfig_EEPROM();

}

void ConfigManager::Reset_to_Default()
{
        memcpy(_Gains, default_Gains,sizeof(default_Gains)); 
        memcpy(_Pids, default_Pids,sizeof(default_Pids));
          for(int i = 0; i < 8 ; i++)
          {
            _SysConfig.ToArray[i] = default_SysConfig.ToArray[i];
          }
          Flags.ToByte = default_SysConfig.Byte_Flags;                          
        
          EEPROM_writeAnything(ADDR_RESET_FLAG,1);  //Set Flag Reset

}


void ConfigManager::GetUpdate()
{
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
                      send_Gains(dtype);
                  
                break;
              case data_type.Pids_Memory: //request PID Config
                      send_Pids(dtype);
                 
              break;
              case data_type.System_Memory:  
                      send_SysConfig(dtype);
              break;
              case data_type.All_Memory: //request all
                      send_Gains((byte)data_type.Gains_Memory);
                      send_Pids((byte)data_type.Pids_Memory);
                      send_SysConfig((byte)data_type.System_Memory);
                     
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
                            receive_SysConfig();
                break;
                case data_type.All_Memory://Send PID Config  
                            receive_Gains();  
                            receive_Pids();
                            receive_SysConfig();
                            
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
                   Write_Gains_EEPROM();
              break;
              case data_type.Pids_Eeprom:
                   Write_Pids_EEPROM();
              break;
              case data_type.System_Eeprom:
                   Write_SysConfig_EEPROM();

              break;
              case data_type.All_Eeprom:
                   Write_All_EEPROM();
                    
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
                          Read_Gains_EEPROM();
                          send_Gains(dtype);
                    break;  
                     case data_type.Pids_Eeprom:
                         Read_Pids_EEPROM();
                         send_Pids(dtype);
                    break; 
                    case data_type.System_Eeprom:
                          Read_SysConfig_EEPROM();
                          send_SysConfig(dtype);
                    case data_type.All_Eeprom:
                          Read_All_EEPROM();
                          send_Gains((byte)data_type.Gains_Eeprom);
                          send_Pids((byte)data_type.Pids_Eeprom);
                          send_SysConfig((byte)data_type.System_Eeprom);  
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
                              Reset_to_Default();
                              send_Gains((byte)data_type.Gains_Memory);
                              send_Pids((byte)data_type.Pids_Memory);
                              send_SysConfig((byte)data_type.System_Memory);
                        break; 
                        
                        default:
                        break;

                  }
    }
    
  } 
  
}



