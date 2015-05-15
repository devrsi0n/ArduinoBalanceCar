#include <MPU6050.h>
#include <TimerOne.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <PID_v1.h>
#include "BalanceCar.h"
#include "MY_EEPROM.h"
#include "StatesMachine.h"
#include "Angle.h"
#include "Motors.h"
#include "bluetooth.h"


void setup(void)
{
    // initEEPROM(); // initialize EEPROM if need
    initMotors();
    initEncoders();
    pinMode(RUNNING_LIGHT_PIN, OUTPUT); // set a light pin represent controller is running.

    TWBR = ((16000000 / 500000) - 16) / 2;    // setup i2c clock to 500kHz
    Wire.begin();
    Serial.begin(115200);
    Serial3.begin(115200); // bluetooth use serial port 3

    accelgyro.initialize();
    initAnglePID();

    Timer1.initialize(2000); // set timer interruption time to 2ms(1us * 2000).
    Timer1.attachInterrupt(timerIsr); // start timer

    getAnglePID();
    getSpeedPID();
    getMotorDeadVal();
}

void loop(void)
{
    runningLight();

    // printAngle();    // uncomment to porint car's angle

    if(Serial3.available() && btCommand == 't') // 't' ---> stop interrupts and enter the arguments adjust state
    {
        Timer1.detachInterrupt(); // stop timer1
        detachInterrupt(4); // close external interrupt #4
        detachInterrupt(5);

        Serial3.println(btCommand);
        btCommand = 0;

        argumentsAdjust(); // change to argsAdjust state
        argsAdjustState();

        Timer1.initialize(2000); // set timer to 2ms(1000 x 1us).
        Timer1.attachInterrupt(timerIsr);
        attachInterrupt(4, encoderLeft,  FALLING);
        attachInterrupt(5, encoderRight, FALLING);
    }
}

void timerIsr(void)
{
    sei(); // to open global interrupt bit.(default close)

    statesMachine(); // main logic codes write here.
}

/*
* if main function is running, light the pin 13
*/
void runningLight(void)
{
    static int counter = 0;
    static boolean flag = false;

    if(counter++ == 30000)
    {
        counter = 0;
        if (flag == true)
        {
            flag = false;
            digitalWrite(RUNNING_LIGHT_PIN, HIGH);
        }
        else
        {
            flag = true;
            digitalWrite(RUNNING_LIGHT_PIN, LOW);
        }
    }
}

void printAngle(void)
{
    static int counter = 0;

    if(counter++ == 1000)
    {
        counter = 0;
        // Serial.print(original_angle);
        // Serial.print(",");
        Serial.print(klm_angle);
        Serial.print(",");
        Serial.println(board_angle);

        // Serial.print(original_gyro);
        // Serial.print(",");
        // Serial.println(board_gyro);
    }
}

/*
* get angle control PID args from EEPROM or macros
*/
void getAnglePID(void)
{
    int saved = readIntFromEEPROM(EEPROM_ANGLE_PID_SAVED_ADDR);    // get saved flag from EEPROM

    if(saved == ANGLE_PID_SAVED)                            // check if PID has been saved to EEPROM
    {
        int nP = readIntFromEEPROM(EEPROM_ANGLE_P_ADDR);  // read P from EEPROM
        CarArgs.angleCtrlP = nP / 100.0;
        int nI = readIntFromEEPROM(EEPROM_ANGLE_I_ADDR);  // read I from EEPROM
        CarArgs.angleCtrlI = nI / 100.0;
        int nD = readIntFromEEPROM(EEPROM_ANGLE_D_ADDR);  // read D from EEPROM
        CarArgs.angleCtrlD = nD / 100.0;
    }
    else
    {
        CarArgs.angleCtrlP = ANGLE_P;                      // use static value
        CarArgs.angleCtrlI = ANGLE_I;                      // use static value
        CarArgs.angleCtrlD = ANGLE_D;                      // use static value
    }
}

/*
* get angle speed PID args from EEPROM or macros
*/
void getSpeedPID(void)
{
    int saved = readIntFromEEPROM(EEPROM_SPEED_PID_SAVED_ADDR);

    if(saved == SPEED_PID_SAVED)
    {
        int nP = readIntFromEEPROM(EEPROM_SPEED_P_ADDR);
        CarArgs.speedCtrlP = nP / 100.0;

        int nI = readIntFromEEPROM(EEPROM_SPEED_I_ADDR);
        CarArgs.speedCtrlI = nI / 100.0;

        int nD = readIntFromEEPROM(EEPROM_SPEED_D_ADDR);
        CarArgs.speedCtrlD = nD / 100.0;
    }
    else
    {
        CarArgs.speedCtrlP = SPEED_P;
        CarArgs.speedCtrlI = SPEED_I;
        CarArgs.speedCtrlD = SPEED_D;
    }
}

/*
* get dead value from EEPROM or macros
*/
void getMotorDeadVal(void)
{
    int saved = readIntFromEEPROM(EEPROM_MOTOR_DEAD_VAL_SAVED_ADDR);

    if (saved == MOTOR_DEAD_VAL_SAVED)
    {
        int deadVal = readIntFromEEPROM(EEPROM_MOTOR_DEAD_VAL_ADDR);
        CarArgs.motorDeadVal = deadVal / 100.0;
    }
    else
        CarArgs.motorDeadVal = MOTOR_OUT_DEAD_VAL;
}

