#include "EEPROM.h"

void writeIntToEEPROM(int address, int value)
{
    byte low = value & 0xFF;
    byte high = value >> 8;

    EEPROM.write(address, low);
    EEPROM.write(address + 1, high);
}

int readIntFromEEPROM(int address)
{
    byte low = EEPROM.read(address);
    byte high = EEPROM.read(address + 1);
    int value = (high << 8) + low;
    return value;
}

/*
 * if EEPROM is not initialized, clear all EEPROM data
 */
void initEEPROM(void)
{
    if (readIntFromEEPROM(EEPROM_ANGLE_PID_SAVED_ADDR) == ANGLE_PID_SAVED) // Not clear EEPROM if args already write into EEPROM, otherwise clear EEPROM
        return;

    int i;
    for(i = 0; i < 4096; i++)
    {
        EEPROM.write(i, 0);
    }
}

