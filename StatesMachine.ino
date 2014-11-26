#include "StatesMachine.h"

void statesMachine(void)
{
    if(Serial3.available())
    {
        btCommand = Serial3.read();
        if(btCommand == 'r')
        {
            bluetoothCtrlReady();
            btCommand = 0;
        }
        else if (btCommand == 'l')
        {
            lockCar();
            btCommand = 0;
        }
        else if(btCommand == 'u')
        {
            unlockCar();
            btCommand = 0;
        }
    }
    if(next_state == emergencyBrake && board_angle >= -2.0 && board_angle <= 2.0)
    {
        static int counter = 0;
        if(board_angle > 2 || board_angle < -2)
            counter = 0;
        if(counter++ == 2000) // hold level and wait for 2 second
        {
            boardLevel();
            counter = 0;
        }
    }
    if(board_angle <= -15 || board_angle >= 15)
    {
        angleOutOfRange();
    }


    switch(next_state)
    {
        case standBalance:
            standBalanceState();
            break;
        case bluetoothCtrl:
            bluetoothCtrlState();
            break;
        case emergencyBrake:
            emergencyBrakeState();
            break;
        case lockIn:
            lockInState();
            break;
        default:
            Serial3.println("statesSeletor error!");
            break;
    }
}

void boardLevel(void)
{
    curr_state = next_state;
    if(curr_state == emergencyBrake)
        next_state = standBalance;
}

void bluetoothCtrlReady(void)
{
    curr_state = next_state;
    if(curr_state == standBalance)
        next_state = bluetoothCtrl;
}

void angleOutOfRange(void)
{
    curr_state = next_state;
    if(curr_state == standBalance || curr_state == bluetoothCtrl)
        next_state = emergencyBrake;
}

void lockCar(void)
{
    curr_state = next_state;
    if(curr_state == standBalance || curr_state == bluetoothCtrl)
        next_state = lockIn;
}

void unlockCar(void)
{
    curr_state = next_state;
    if(curr_state == lockIn)
        next_state = emergencyBrake;
}

void argumentsAdjust(void)
{
    curr_state = next_state;
    if(curr_state == standBalance || curr_state == emergencyBrake || curr_state == lockIn)
        next_state = argsAdjust;
}

void standBalanceState(void)
{
    static int i = 0;

    switch(i)
    {
        case 0:
            readSampleMPU6050();
            break;
        case 1:
            getOriginalAngleGyro();
            angleFilter();
            break;
        case 2:
            sampleSpeed();
            speedCtrl();
            speedCtrlOutput();
            break;
        case 3:
            directionCtrl();
            directionCtrlOutput();
            break;
        case 4:
            motorsOutput();
            break;
        default:
            Serial.println("standBalanceState error!");
            while(1); //
    }

    if(++i == 5)
        i = 0;
}

void bluetoothCtrlState(void)
{
    angle_ctrl_output = 0;
    speed_ctrl_output = 0;
    direction_ctrl_output = 0;
    motorsOutput();
}

void emergencyBrakeState(void)
{
    angle_ctrl_output = 0;
    speed_ctrl_output = 0;
    direction_ctrl_output = 0;
    motorsOutput();
}

void lockInState(void)
{
    angle_ctrl_output = 0;
    speed_ctrl_output = 0;
    direction_ctrl_output = 0;
    motorsOutput();
}

void argsAdjustState(void)
{
    angle_ctrl_output = 0;
    speed_ctrl_output = 0;
    direction_ctrl_output = 0;
    motorsOutput();

    while(!Serial3.available())
        ;
    btCommand = Serial3.read();
    while(btCommand != 'p' && btCommand != 'i' && btCommand != 'v')
    {
        btCommand = Serial3.read();
        while(!Serial3.available())
            ;
    }
    Serial3.println(btCommand);
    argsAdjustSaveDate(btCommand);
    getAnglePD();
    // getSpeedPI();
    // getMotorDeadVal();
    returnArgsDate();

    while(!Serial3.available())
        ;
    while((btCommand = Serial3.read()) != 'f')
    {
        while(!Serial3.available())
            ;
    }
    Serial3.println(btCommand);
    btCommand = 0;

#ifdef _PRINT_ANGLE_PD
    Serial.print("angleP:");
    Serial.println(CarArgs.angleCtrlP);
    Serial.print("angleD:");
    Serial.println(CarArgs.angleCtrlD);
    Serial.print("speedP:");
    Serial.println(CarArgs.speedCtrlP);
    Serial.print("speedI:");
    Serial.println(CarArgs.speedCtrlI);
    Serial.print("motorDeadVal:");
    Serial.println(CarArgs.motorDeadVal);
#endif


}
