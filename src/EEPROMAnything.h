
#include <Arduino.h>  // for type definitions
#ifdef _VARIANT_ARDUINO_DUE_X_
#include "DueFS/DueFlashStorage.h"
#define EEPROM dueFlashStorage
DueFlashStorage dueFlashStorage;
#else
#include <EEPROM.h>
#endif


template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    
          EEPROM.write(ee++, *p++);
          //dueFlashStorage.write(ee++,*p++);
   
     return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
        //*p++ = dueFlashStorage.read(ee++);
        *p++ = EEPROM.read(ee++);

    return i;
  
}