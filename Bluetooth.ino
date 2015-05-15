#include "Bluetooth.h"

/*
* adjust argments by bluetooth and save data to EEPROM
*/
void argsAdjustSaveData(void)
{
    if(btCommand == 'p') // adjust angle args
    {
        btCommand = 0;
        Serial3.println("Now you can input the pid of angle:");

        int angleP = str2int();
        writeIntToEEPROM(EEPROM_ANGLE_P_ADDR, angleP);

        int angleI = str2int();
        writeIntToEEPROM(EEPROM_ANGLE_I_ADDR, angleI);

        int angleD = str2int();
        writeIntToEEPROM(EEPROM_ANGLE_D_ADDR, angleD);

        writeIntToEEPROM(EEPROM_ANGLE_PID_SAVED_ADDR, ANGLE_PID_SAVED); // set a flag to EEPROM

        Serial3.println("success!");
    }
    else if(btCommand == 'i') // adjust speed args
    {
        btCommand = 0;
        Serial3.println("Now you can input the pid of speed:");

        int speedP = str2int();
        writeIntToEEPROM(EEPROM_SPEED_P_ADDR, speedP);

        int speedI = str2int();
        writeIntToEEPROM(EEPROM_SPEED_I_ADDR, speedI);

        int speedD = str2int();
        writeIntToEEPROM(EEPROM_SPEED_D_ADDR, speedD);

        writeIntToEEPROM(EEPROM_SPEED_PID_SAVED_ADDR, SPEED_PID_SAVED);

        Serial3.println("success!");
    }
    else if(btCommand  == 'm') // adjust motor dead value
    {
        btCommand = 0;

        int motorDeadValue = str2int();
        writeIntToEEPROM(EEPROM_MOTOR_DEAD_VAL_ADDR, motorDeadValue);

        writeIntToEEPROM(EEPROM_MOTOR_DEAD_VAL_SAVED_ADDR, MOTOR_DEAD_VAL_SAVED);

        Serial3.println("success!");
    }
}

void sendArgsData(void)
{
    float angleP = CarArgs.angleCtrlP;//the real value you set
    float angleI = CarArgs.angleCtrlI;
    float angleD = CarArgs.angleCtrlD;

    float speedP = CarArgs.speedCtrlP;
    float speedI = CarArgs.speedCtrlI;
    float speedD = CarArgs.speedCtrlD;

    float deadVal= CarArgs.motorDeadVal;

    // print angle PID args
    Serial3.println("The real value you input:");
    Serial3.print("angle\nP:");
    Serial3.print(angleP);
    Serial3.print("\tI:");
    Serial3.print(angleI);
    Serial3.print("\tD:");
    Serial3.println(angleD);

    // print speed PID args
    Serial3.print("speed\nP:");
    Serial3.print(speedP);
    Serial3.print("\tI:");
    Serial3.print(speedI);
    Serial3.print("\tD:");
    Serial3.println(speedD);

    // print motor dead value
    Serial3.print("mV:");
    Serial3.println(deadVal);
}

void sendCarSpeed(void)
{
    float speedL = rpm_left;
    float speedR = rpm_right;

    Serial3.print("Set speed:");
    Serial3.println(set_car_speed);
    Serial3.print("L:");
    Serial3.println(speedL);
    Serial3.print("R:");
    Serial3.println(speedR);
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
        else
        {
            Serial3.println(" is not a digit number!");
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
