#include "Bluetooth.h"

/*
* adjust argments by bluetooth and save data to EEPROM
*/
void argsAdjustSaveData(char btCommand)
{
    if(btCommand == 'P') // adjust angle args P&D
    {
        btCommand = 0; // avoid function re-execution

        int angleP = str2int();
        writeIntToEEPROM(EEPROM_ANGLE_P_ADDR, angleP);
        int angleD = str2int();
        writeIntToEEPROM(EEPROM_ANGLE_D_ADDR, angleD);
        writeIntToEEPROM(EEPROM_ANGLE_PD_SAVED_ADDR, ANGLE_PD_SAVED); // set a flag to EEPROM

        Serial3.println('s'); // mean save date success.
    }
    else if(btCommand == 'i') // adjust speed args P&I
    {
        btCommand = 0;

        int speedP = str2int();
        writeIntToEEPROM(EEPROM_SPEED_P_ADDR, speedP);
        int speedI = str2int();
        writeIntToEEPROM(EEPROM_SPEED_I_ADDR, speedI);
        writeIntToEEPROM(EEPROM_SPEED_PI_SAVED_ADDR, SPEED_PI_SAVED);

        Serial3.println('s'); // mean save date success.
    }
    else if(btCommand  == 'v') // adjust motor dead value
    {
        btCommand = 0;

        int motorDeadValue = str2int();
        writeIntToEEPROM(EEPROM_MOTOR_DEAD_VAL_ADDR, motorDeadValue);
        writeIntToEEPROM(EEPROM_MOTOR_DEAD_VAL_SAVED_ADDR, MOTOR_DEAD_VAL_SAVED);

        Serial3.println('s'); // mean save date success.
    }
}

void sendArgsData(void)
{
    // int angleP = (int)(CarArgs.angleCtrlP * 100);
    // int angleD = (int)(CarArgs.angleCtrlD * 100);
    // int speedP = (int)(CarArgs.speedCtrlP * 100);
    // int speedI = (int)(CarArgs.speedCtrlI * 100);
    // int deadVal= (int)(CarArgs.motorDeadVal * 100);

    Serial3.print("P:");
    Serial3.println(CarArgs.angleCtrlP, 2);
    Serial3.print("D:");
    Serial3.println(CarArgs.angleCtrlD, 2);
    Serial3.print("P:");
    Serial3.println(CarArgs.speedCtrlP, 2);
    Serial3.print("I:");
    Serial3.println(CarArgs.speedCtrlI, 2);
    Serial3.print("V:");
    Serial3.println(CarArgs.motorDeadVal, 2);
}

void sendCarSpeed(void)
{
    // int speedL = (int)rpm_left;
    // int speedR = (int)rpm_right;

    Serial3.print("L:");
    Serial3.println(rpm_left, 2);
    Serial3.print("R:");
    Serial3.println(rpm_right, 2);
    Serial3.print("Set speed:");
    Serial3.println(set_car_speed);
}

/*
* read char from serial port and convert string to int number.
*/
static int str2int(void)
{
    byte buffer[10] = {0};
    char tempChar   = 0;
    int counter     = 0;
    int counter2    = 0;
    int index       = 0;
    int mask = 1;
    int result = 0;

    while(!Serial3.available())
        ;// Wait
    while((tempChar = (char)Serial3.read()) != '%')
    {
        Serial3.print(tempChar);
        if(tempChar >= '0' && tempChar <= '9')
        {
            buffer[counter++] = tempChar;
        }
        while(!Serial3.available())
            ;
    }
    Serial3.println(); // print a new line
    --counter;
    for(index = 0; counter >= 0; --counter, ++index)
    {
        mask = 1;
        for(counter2 = counter; counter2 > 0; --counter2)
            mask *= 10;
        result += (int)(buffer[index] - '0') * mask;
    }

    return result;
}
