#include <MPU6050.h>
#include <TimerOne.h>
#include <Wire.h>
#include <I2Cdev.h>
#include "BalanceCar.h"
#include "MY_EEPROM.h"
#include "StatesMachine.h"
#include "Angle.h"
#include "Motors.h"
#include "bluetooth.h"
#include "PID.h"


void setup(void)
{
    // initEEPROM(); // clear all of EEPROM data, comment it if need
    initMotors();//define related pin
    initEncoders();//define encoder interrupt pin
    pinMode(RUNNING_LIGHT_PIN, OUTPUT); // set a light pin represent controller is running.

    TWBR = ((16000000 / 500000) - 16) / 2;    // setup i2c clock to 500kHz TWBR = 8
    Wire.begin();
    Serial.begin(115200);
    Serial3.begin(115200); // bluetooth use serial port 3

    getAnglePID();//get the value of pid from related register
    getSpeedPID();
    getMotorDeadVal();

    accelgyro.initialize();

    Timer1.initialize(2000); // set timer interruption time to 2ms(1us * 2000).
    Timer1.attachInterrupt(timerIsr); // start timer
}

void loop(void)
{
    runningLight();

    //printAngle();    // uncomment it to porint car's angle

    if(Serial3.available() && ((btCommand=Serial3.read()) == 't'))// 't' ---> stop interrupts and enter the arguments adjust state
    {
        Timer1.detachInterrupt(); // stop timer1
        detachInterrupt(4); // close external interrupt #4
        detachInterrupt(5);

        Serial3.println(btCommand);
        Serial3.println("Now enter the parameterAdjustState!");
        btCommand = 0;

        parameterAdjustReady(); // switch to argsAdjust state
        parameterAdjustState();

        Timer1.initialize(2000); // set timer to 2ms(1000 x 1us).
        Timer1.attachInterrupt(timerIsr);
        attachInterrupt(4, encoderLeft,  FALLING); // reopen external interrupt #4
        attachInterrupt(5, encoderRight, FALLING);
    }
}

void timerIsr(void)
{
    sei(); // to open global interrupt bit.(default is closed)

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
        // Serial.print(klm_angle);
        // Serial.print(",");
        Serial.println(board_angle);

        // Serial.print(original_gyro);
        // Serial.print(",");
        // Serial.println(board_gyro);

        // Serial3.println(angle_ctrl_output);
        // Serial.println(CarArgs.angleCtrlI);
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
        CarArgs.angleCtrlP = nP / 1000.0;
        int nI = readIntFromEEPROM(EEPROM_ANGLE_I_ADDR);  // read I from EEPROM
        CarArgs.angleCtrlI = nI / 1000.0;
        int nD = readIntFromEEPROM(EEPROM_ANGLE_D_ADDR);  // read D from EEPROM
        CarArgs.angleCtrlD = nD / 1000.0;
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
        CarArgs.speedCtrlP = nP / 1000.0;

        int nI = readIntFromEEPROM(EEPROM_SPEED_I_ADDR);
        CarArgs.speedCtrlI = nI / 1000.0;

        int nD = readIntFromEEPROM(EEPROM_SPEED_D_ADDR);
        CarArgs.speedCtrlD = nD / 1000.0;
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
        CarArgs.motorDeadVal = deadVal / 1000.0;
    }
    else
        CarArgs.motorDeadVal = MOTOR_OUT_DEAD_VAL;
}

