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
    // clearEEPROM(); // uncomment it to clear EEPROM data if need

    getAnglePD();
    getSpeedPI();
    getMotorDeadVal();

    initMotors();
    initEncoders();
    pinMode(DEBUG_LIGHT_PIN, OUTPUT);

    TWBR = ((16000000 / 500000) - 16) / 2;    // setup i2c clock to 500kHz
    Wire.begin();
    Serial.begin(115200);
    Serial3.begin(115200); // bluetooth use serial port 3

    accelgyro.initialize();
    initAnglePID();

    Timer1.initialize(2000); // set timer interruption time to 2ms(1us * 2000).
    Timer1.attachInterrupt(timerIsr); // start timer
}

void loop(void)
{
    debugLight();
    // printAngle();    // uncomment to porint car's angle
    if(btCommand == 't') // 't' ---> stop interrupts and enter the arguments adjust state
    {
        Timer1.detachInterrupt();
        detachInterrupt(4); // close external interrupt #4
        detachInterrupt(5);
        Serial3.println(btCommand);
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
    sei();
    statesMachine();
}


void debugLight(void)
{
    static int counter = 0;
    static boolean flag = false;
    if(counter++ == 30000)
    {
        counter = 0;
        if (flag == true)
            flag = false;
        else
            flag = true;
    }
    if(flag)
    {
        // Serial.println("t");
        digitalWrite(DEBUG_LIGHT_PIN, HIGH);
    }
    else
    {
        digitalWrite(DEBUG_LIGHT_PIN, LOW);
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
        Serial.println(board_angle);
        // Serial.println(angle_input);
        // Serial.println(CarArgs.angleCtrlP);
    }
}

/*
* get angle control PID args(P & D)
*/
void getAnglePD(void)
{
    int saved = readIntFromEEPROM(EEPROM_ANGLE_PD_SAVED_ADDR);    // get saved flag from EEPROM
    if(saved == ANGLE_PD_SAVED)                            // check if PD has been saved to EEPROM
    {
        int Pi = readIntFromEEPROM(EEPROM_ANGLE_P_ADDR);  // read P from EEPROM
        CarArgs.angleCtrlP = Pi / 100.0;
        int Di = readIntFromEEPROM(EEPROM_ANGLE_D_ADDR);  // read D from EEPROM
        CarArgs.angleCtrlD = Di / 100.0;
    }
    else
    {
        CarArgs.angleCtrlP = ANGLE_P;                      // use static value
        CarArgs.angleCtrlD = ANGLE_D;                      // use static value
    }
}

void getSpeedPI(void)
{
    int saved = readIntFromEEPROM(EEPROM_SPEED_PI_SAVED_ADDR);
    if(saved == SPEED_PI_SAVED)
    {
        int Ps = readIntFromEEPROM(EEPROM_SPEED_P_ADDR);
        CarArgs.speedCtrlP = Ps / 100.0;
        int Is = readIntFromEEPROM(EEPROM_SPEED_I_ADDR);
        CarArgs.speedCtrlI = Is / 100.0;
    }
    else
    {
        CarArgs.speedCtrlP = SPEED_P;
        CarArgs.speedCtrlI = SPEED_I;
    }
}

void getMotorDeadVal(void)
{
    int saved = readIntFromEEPROM(EEPROM_MOTOR_DEAD_VAL_SAVED_ADDR);
    if (saved == MOTOR_DEAD_VAL_SAVED)
    {
        int deadVal = readIntFromEEPROM(EEPROM_MOTOR_DEAD_VAL_ADDR);
        CarArgs.motorDeadVal = deadVal / 100.0;
    }
    else
        CarArgs.motorDeadVal = MOTOR_OUT_DEAD_VAL;       //use static value
}

