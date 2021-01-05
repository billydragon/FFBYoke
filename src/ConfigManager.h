#ifndef CONFIGMANAGER_h
#define CONFIGMANAGER_h
#include "Arduino.h"
//#include "Joystick.h"

#define ADDR_RESET_FLAG				2					
#define ADDR_GAINS_START			4		//(ADDR_OFFSET_X + 4)				
#define ADDR_GAINS_LEN				26		// 13 *2
#define ADDR_PIDS_START				34		//(ADDR_GAINS_START + ADDR_GAINS_LEN)
#define ADDR_PIDS_LEN				40		//(4 * 5) *2	sizeof(float) 4Byte * 5 
#define ADDR_START_SYSCONFIG		78		//(ADDR_PIDS_START + ADDR_PIDS_LEN)
#define ADDR_SYSCONTROL_LEN			8		//

struct DATA_TYPE 
{
	enum : uint8_t
	{
		Gains_Memory = 0x01,
		Pids_Memory,
		System_Memory,
		All_Memory,
		Gains_Eeprom,
		Pids_Eeprom,
		System_Eeprom,
		All_Eeprom,
		Control_CMD,
		Reset_Default
	};
};

struct COMMAND_TYPE 
{
	enum : uint8_t
	{
		Read_Memory = 0x10,
		Write_Memory,
		Load_Eeprom,
		Save_Eeprom,
		Control
	};

};

struct COMMAND_HEADER
{
	uint8_t Command;
	uint8_t Data_Type;
	uint8_t Axis;
	uint8_t Start_Index;
	uint8_t Lenght;
};

union COMMANDS
{
	COMMAND_HEADER Header;
	uint8_t cmd_Bytes[sizeof(COMMAND_HEADER)];
};


struct GAIN
{
	uint8_t totalGain;         
	uint8_t constantGain;    
	uint8_t rampGain;          
	uint8_t squareGain;        
	uint8_t sineGain;        
	uint8_t triangleGain;     
	uint8_t sawtoothdownGain;  
	uint8_t sawtoothupGain;   
	uint8_t springGain;        
	uint8_t damperGain; 
	uint8_t inertiaGain;     
	uint8_t frictionGain;      
	uint8_t customGain;     
};

union GAINS_CONFIG
{
	GAIN Gain;
	uint8_t GainsArray[sizeof(GAIN)];
};


struct PID
{
	float MaxOutput;
	float SampleTime;
	float Kp;
	float Ki;
	float Kd;
};//pid;
	
union PIDS_CONFIG
{
	PID Pid;
	float PidsArray[sizeof(PID)];
};

struct CONFIG
{
		uint8_t Motor_Inv_X;
		uint8_t Motor_Inv_Y;
		uint8_t Swap_XY_Force;
		uint8_t Auto_Calibration;
		uint8_t Motor_Dir_Delay;
		uint8_t Reserve1;
		uint8_t Reserve2;
		uint8_t Reserve3;
		
};

union SYSTEM_CONFIGS
{
	CONFIG Byte;
	uint8_t ToByteArray[sizeof(CONFIG)];
};



union int32_union
{
    uint8_t     uiBytes[sizeof(int32_t)];
    uint32_t     ui32Value;
};

union float_union
{
    uint8_t   uiBytes[sizeof(float)];
    float     fValue;
};

class ConfigManager
{
private:
        
        void send_Gains(byte dt);
        void send_SysConfig(byte dt);
        void send_Pids(byte dt);
        void receive_Gains();
        void receive_Pids();
        void receive_SysConfig();
		void Reset_to_Default();
		void Read_Gains_EEPROM();
		void Read_Pids_EEPROM();
		void Read_SysConfig_EEPROM();
		void Read_All_EEPROM();	
		void Write_Gains_EEPROM();
		void Write_Pids_EEPROM();
		void Write_SysConfig_EEPROM();
		void Write_All_EEPROM();

public:
    GAINS_CONFIG _Gains[2];
	PIDS_CONFIG _Pids[2];
	SYSTEM_CONFIGS _SysConfig;
    ConfigManager();
    ~ConfigManager();
	
	void begin();		
    void GetUpdate();
};

#endif