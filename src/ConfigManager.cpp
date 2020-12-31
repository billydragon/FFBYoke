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
const System_Control default_SysCtrl = System_Control{0,0,0,0,0,0,0,0};



template <class T> long int _writeAnything(T& valuebt)
{
long i = Serial.write((byte *) &valuebt, sizeof(valuebt));
  
  return i;
}


ConfigManager::ConfigManager()
{
  first_run = true;
}

void ConfigManager::begin() 
{
      
      if(first_run == true)
      {
        EEPROM_readAnything(ADDR_GAINS_START,_Gains);
        
        EEPROM_readAnything(ADDR_PIDS_START,_Pids);
      
        EEPROM_readAnything(ADDR_START_SYSCONTROL,_SysCtrl.ToByte); 
      
        first_run = false;
      }
      
}

ConfigManager::~ConfigManager()
{
}

void ConfigManager::send_gains(byte dt)
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
            _writeAnything(_Gains[i].ToBytes[j]);   
            //delay(20);
          }
          
        }
}

void ConfigManager::send_sys_control(byte dt)
{
            //byte cmd[3];
            CMD.Command = command_type.Read_Memory;
            CMD.Data_Type = dt;
            CMD.Axis = 0;
            CMD.Start_Index = 0;
            CMD.Lenght  = 1;
            _writeAnything(CMD.toBytes);
            _writeAnything(_SysCtrl.ToByte);
            
            //delay(20);
      
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
            CMD.Lenght = 4;
              _writeAnything(CMD.toBytes); 
              _writeAnything(_Pids[i].ToFloat[j]);
              //delay(20);
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
                _Gains[idx].ToBytes[pos + i] = c;

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

            _Pids[idx].ToFloat[pos + i] = f_union.fValue;
                 
          }
}

void ConfigManager::receive_Sys_control()
{
      byte idx = Serial.read();           // 3 Axis index
      byte pos = Serial.read();           // 4
      byte cmdlen = Serial.read();        // 5  lenght of data  
      _SysCtrl.ToByte = Serial.read();

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
                      send_gains(dtype);
                  
                break;
              case data_type.Pids_Memory: //request PID Config
                      send_Pids(dtype);
                 
              break;
              case data_type.System_Memory:  
                      send_sys_control(dtype);
              break;
              case data_type.All_Memory: //request all
                      send_gains(data_type.Gains_Memory);
                      send_Pids(data_type.Pids_Memory);
                      send_sys_control(data_type.System_Memory);
                     
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
                     
                    EEPROM_writeAnything(ADDR_GAINS_START,_Gains);
                    delay(10); 
                    //memcpy(_gains,uGains,sizeof(_gains));
              break;
              case data_type.Pids_Eeprom:
                    EEPROM_writeAnything(ADDR_PIDS_START,_Pids);
                     delay(10);
                     // memcpy(_pids,uPids,sizeof(_pids));
              break;
              case data_type.System_Eeprom:
                    EEPROM_writeAnything(ADDR_START_SYSCONTROL,_SysCtrl.ToByte ); 
                    delay(10);
                    //_sys_ctr = uSysControl;

              break;
              case data_type.All_Eeprom:
                    EEPROM_writeAnything(ADDR_GAINS_START,_Gains);
                    delay(10); 
                    EEPROM_writeAnything(ADDR_PIDS_START,_Pids);
                    delay(10);
                    EEPROM_writeAnything(ADDR_START_SYSCONTROL,_SysCtrl.ToByte ); 
                    delay(10);
                    

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
                          EEPROM_readAnything(ADDR_GAINS_START,_Gains); 
                           //delay(20);     
                          send_gains(dtype);
                    break;  
                     case data_type.Pids_Eeprom:
                          EEPROM_readAnything(ADDR_PIDS_START,_Pids);
                           //delay(20);  
                          send_Pids(dtype);
                    break; 
                    case data_type.System_Eeprom:
                          EEPROM_readAnything(ADDR_START_SYSCONTROL,_SysCtrl.ToByte); 
                          send_sys_control(dtype); 
                    case data_type.All_Eeprom:
                         //load_all_eeprom();
                         EEPROM_readAnything(ADDR_GAINS_START,_Gains);
                         //delay(20);
                         send_gains(data_type.Gains_Eeprom);
                        EEPROM_readAnything(ADDR_PIDS_START,_Pids);
                        //delay(20);
                        send_Pids(data_type.Pids_Eeprom);
                        EEPROM_readAnything(ADDR_START_SYSCONTROL,_SysCtrl.ToByte); 
                        //delay(20);
                        send_sys_control(data_type.System_Eeprom);
                         
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
                              
                              memcpy(_Gains, default_Gains,sizeof(default_Gains));
                              //delay(20);    
                              send_gains(data_type.Gains_Memory);
                              memcpy(_Pids, default_Pids,sizeof(default_Pids));
                              //delay(20);  
                              send_Pids(data_type.Pids_Memory);
                              _SysCtrl = default_SysCtrl;
                              send_sys_control(data_type.System_Memory);
                              
                        break; 
                        
                        default:
                        break;

                  }
    }
    
  }

       
  
}



