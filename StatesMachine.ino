#include "StatesMachine.h"

/*******************************************************
* States machine: when reach to a condition, switch
*   to another state. But, there is a special state, if
*   enter into argsAdjustState, stop timer and receved
*   args and write args into the EEPROM.
*******************************************************/

void statesMachine(void)
{
    if(Serial3.available())
    {
        btCommand = Serial3.read();
        if(btCommand == 'r') // 'r' ---> bluetooth control "ready"
        {
            Serial3.println(btCommand); // let us know whether the bluetooth command come into controller.
            bluetoothCtrlReady();
            btCommand = 0;
        }
        else if (btCommand == 'l') // 'l' ---> "lock" the car
        {
            Serial3.println(btCommand);
            lockCar();
            btCommand = 0;
        }
        else if(next_state == lockIn && btCommand == 'u') // 'u' ---> 'unlock' the car
        {
            Serial3.println(btCommand);
            unlockCar();
            btCommand = 0;
        }
    }
    if(next_state == emergencyBrake && board_angle >= LEVEL_ANGLE_MIN && board_angle <= LEVEL_ANGLE_MAX)
    {
        static int counter = 0;
        if(board_angle > LEVEL_ANGLE_MAX || board_angle < LEVEL_ANGLE_MIN)
            counter = 0;
        if(counter++ == 1000) // hold level and wait for 2 second(2ms x 1000).
        {
            boardLevel();
            counter = 0;
        }
    }
    if(board_angle <= EMERGENCY_BRAKE_ANGLE_MIN || board_angle >= EMERGENCY_BRAKE_ANGLE_MAX)
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
        Serial3.println("States seletor error!");
        break;
    }
}

/*
* if current state is emgencyBrakeState and angle is [-5, 5](wait for a couple seconds),
* swtich state to standBalance state.
*/
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

/*
* if angle beyond (-15, 15) degree, swtich state to emergencyBrake state.
*/
void angleOutOfRange(void)
{
    curr_state = next_state;
    if(curr_state == standBalance || curr_state == bluetoothCtrl)
        next_state = emergencyBrake;
}

/*
* if bluetooth send char 'l', swtich state to lockIn state.
*/
void lockCar(void)
{
    curr_state = next_state;
    if(curr_state == standBalance || curr_state == bluetoothCtrl)
        next_state = lockIn;
}

/*
* if bluetooth send char 'u', swtich state to emergencyBrake state.
*/
void unlockCar(void)
{
    curr_state = next_state;
    if(curr_state == lockIn)
        next_state = emergencyBrake;
}

/*
* if bluetooth send char 't', swtich state to argsAdjust state.
*/
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
        angleCtrlPID();
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
        if(btCommand == 'e') // mean send pid arguments from arduino by bluetooth
        {
            sendArgsData();
            btCommand = 0;
        }
        else if(btCommand == 'g')
        {
            Serial3.println(board_angle);
            btCommand = 0;
        }
        break;
    default:
        Serial3.println("standBalanceState error!");
        while(1); //stop here
    }

    if(++i >= 5)
        i = 0;
}

void bluetoothCtrlState(void)
{
    static int j = 0;

    switch(j)
    {
    case 0:
        readSampleMPU6050();
        break;
    case 1:
        getOriginalAngleGyro();
        angleFilter();
        angleCtrlPID();
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
        if(btCommand == 'e') // mean send pid arguments from arduino by bluetooth
        {
            Serial3.println(btCommand);
            sendArgsData();
            btCommand = 0;
        }
        else if(btCommand == 'g')
        {
            Serial3.println(btCommand);
            Serial3.println(board_angle);
            btCommand = 0;
        }
        else if(btCommand == 'o')
        {
            Serial3.println(btCommand);
            sendCarSpeed();
            btCommand = 0;
        }
        break;
    default:
        Serial3.println("bluetoothCtrlState error!");
        while(1); //stop here
    }
    if(++j >= 5)
        j = 0;
}

void emergencyBrakeState(void)
{
    static int k = 0;
    static int counter = 0;

    switch(k)
    {
    case 0:
        readSampleMPU6050();
        break;
    case 1:
        getOriginalAngleGyro();
        break;
    case 2:
        angleFilter();
        break;
    case 3:
        // slow down motors pwm value gentle.
        if(angle_ctrl_output > 0)
            --angle_ctrl_output;
        else if(angle_ctrl_output < 0)
            ++angle_ctrl_output;
        if(speed_ctrl_output > 0)
            --speed_ctrl_output;
        else if(speed_ctrl_output < 0)
            ++speed_ctrl_output;
        if(direction_ctrl_output > 0)
            --direction_ctrl_output;
        else if(direction_ctrl_output < 0)
            ++direction_ctrl_output;
        motorsOutput();
        break;
    case 4:
        if(btCommand == 'e')
        {
            btCommand = 0;
            sendArgsData();
        }
        if(++counter == 50) // --> 2 x 5 x 50 = 500ms
        {
            counter = 0;
            Serial3.println(board_angle); // send board's angle per 0.5 second
        }
        break;
    default:
        Serial3.println("emergencyBrakeState error");
        while(1);
    }
    if(++k >= 5)
        k = 0;
}

void lockInState(void)
{
    static int l = 0;
    static int counter = 0;

    switch(l)
    {
    case 0:
        readSampleMPU6050();
        break;
    case 1:
        getOriginalAngleGyro();
        break;
    case 2:
        angleFilter();
        break;
    case 3:
        if(angle_ctrl_output > 0)
            --angle_ctrl_output;
        else if(angle_ctrl_output < 0)
            ++angle_ctrl_output;
        if(speed_ctrl_output > 0)
            --speed_ctrl_output;
        else if(speed_ctrl_output < 0)
            ++speed_ctrl_output;
        if(direction_ctrl_output > 0)
            --direction_ctrl_output;
        else if(direction_ctrl_output < 0)
            ++direction_ctrl_output;
        motorsOutput();
        break;
    case 4:
        if(btCommand == 'e')
        {
            btCommand = 0;
            sendArgsData();
        }
        break;
    default:
        Serial3.println("lockInState error");
        while(1);
    }
    if(++l >= 5)
        l = 0;
}

void argsAdjustState(void)
{
    float left_value  = angle_ctrl_output - speed_ctrl_output - direction_ctrl_output;
    float right_value = angle_ctrl_output - speed_ctrl_output + direction_ctrl_output;

    // slow down motors pwm value gentle.
    while(left_value != 0 || right_value != 0)
    {
        if(left_value > 0)
            --left_value;
        else if(left_value < 0)
            ++left_value;
        if(right_value > 0)
            --right_value;
        else if(right_value < 0)
            ++right_value;
        motorsOutputAdjust(left_value, right_value);
    }

    while(!Serial3.available())
        ;
    btCommand = Serial3.read();
    while(btCommand != 'p' && btCommand != 'i' && btCommand != 'v') // 'p' --> save angle PID args, 'i' --> save speed PID args, 'v' --> save motor dead value
    {
        btCommand = Serial3.read();
        while(!Serial3.available())
            ;
    }
    Serial3.println(btCommand);
    argsAdjustSaveData(btCommand);
    getAnglePD();
    getSpeedPI();
    getMotorDeadVal();
    sendArgsData();

    while(!Serial3.available())
        ;
    while((btCommand = Serial3.read()) != 'f')
    {
        while(!Serial3.available())
            ;
    }
    Serial3.println(btCommand);
    btCommand = 0;
    next_state = emergencyBrake;

#ifdef _PRINT_ARGS
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
