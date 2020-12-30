#ifndef CONFIGMANAGER_h
#define CONFIGMANAGER_h
#include "Arduino.h"
#include "Joystick.h"


#define ADDR_GAINS_START			4		//(ADDR_OFFSET_X + 4)				
#define ADDR_GAINS_LEN				26		// 13 *2
#define ADDR_PIDS_START				34		//(ADDR_GAINS_START + ADDR_GAINS_LEN)
#define ADDR_PIDS_LEN				40		//(4 * 5) *2	sizeof(float) 4Byte * 5 
#define ADDR_START_SYSCONTROL		78		//(ADDR_PIDS_START + ADDR_PIDS_LEN)
#define ADDR_SYSCONTROL_LEN			8		//

typedef struct 
{
enum : byte
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
}DATA_TYPE;


typedef struct 
{
		enum : byte
		{
			Read_Memory = 0x10,
			Write_Memory,
			Load_Eeprom,
			Save_Eeprom,
			Control
			
		};

}COMMAND_TYPE;


typedef union 
{
	struct
	{
		byte Command;
		byte Data_Type;
		byte Axis;
		byte Start_Index;
		byte Lenght;
	};
	byte toBytes[5];
}COMMAND_HEADER;


typedef union 
{
	struct _GAIN
	{
		byte totalGain;         
		byte constantGain;    
		byte rampGain;          
		byte squareGain;        
		byte sineGain;        
		byte triangleGain;     
		byte sawtoothdownGain;  
		byte sawtoothupGain;   
		byte springGain;        
		byte damperGain; 
		byte inertiaGain;     
		byte frictionGain;      
		byte customGain;     
	}gain;
	byte ToBytes[13];
}GainsConfig;

typedef union 
{
	struct _PID
	{
	float MaxOutput;
	float SampleTime;
	float Kp;
	float Ki;
	float Fd;
	}pid;
	float ToFloat[5];
}PidsConfig;

typedef union 
{
	struct CONTROL
	{
		bool Motor_Inv_X;
		bool Motor_Inv_Y;
		bool Swap_XY_Forces;
		bool Auto_Calibration;
		bool Reserv_1;
		bool Reserv_2;
		bool Reserv_3;
		bool Reserv_4;
	}ctrl;
	byte ToByte;
} System_Control;



union int32_union
{
    uint8_t     uiBytes[sizeof( int32_t )];
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
        
        void send_gains();
        void send_sys_control();
        void send_Pids();
        void receive_Gains();
        void receive_Pids();
        void receive_Sys_control();
		bool first_run = true;
		
public:
    GainsConfig _Gains[2];
	PidsConfig _Pids[2];
	System_Control _SysCtrl;
    ConfigManager();
    ~ConfigManager();
	void begin();		
    void GetUpdate();
};



#endif