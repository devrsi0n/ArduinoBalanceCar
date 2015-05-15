#include "StatesMachine.h"

/*******************************************************
* States machine: when reach to a condition, switch
*   to another state. But, there is a special state, if
*   enter into argsAdjustState, stop timer and receved
*   args and write args into the EEPROM.
*******************************************************/

void statesMachine(void)
{
    // if(Serial3.available())
    // {
    //     btCommand = Serial3.read();


        // if(btCommand == 'j') // 
        // {
        //     Serial3.println(btCommand); // let us know whether the bluetooth command write into controller.
        //     parameterAdjustReady();
        //     btCommand = 0;
        // }
        // else 
        if(btCommand == 'a') // 'a' --->  'advance':go forward
        {
            Serial3.println(btCommand);
            goForward();
            btCommand = 0;
        }
        else if(btCommand == 'd') // 'd' ---> "direction": control the direction of the car
        {
            Serial3.println(btCommand);
            turningCircle();
            btCommand = 0;
        }

    //}

    if(next_state == lockIn && board_angle >= LEVEL_ANGLE_MIN && board_angle <= LEVEL_ANGLE_MAX)
    {
        static int counter = 0;
        if(board_angle > LEVEL_ANGLE_MAX || board_angle < LEVEL_ANGLE_MIN)
            counter = 0;
        if(counter++ == 1000) // hold level and wait for 2 second(2ms x 1000).
        {
            boardLevel();//state change from lockIn to standBalance
            counter = 0;
        }
    }
    if(board_angle <= EMERGENCY_BRAKE_ANGLE_MIN || board_angle >= EMERGENCY_BRAKE_ANGLE_MAX)
    {
        angleOutOfRange();//while the angle is out of range
    }

    switch(next_state)
    {
    case standBalance:
        standBalanceState();//a state of the car to stand
        break;
    case parameterAdjust:
        parameterAdjustState();//adjust the parameter about PID
        break;
    case advanceCtrl:
        advanceState();//a state of the car to go forward
        break;
    case directionCtrl:
        directionCtrlState();//the car to turning circle
        break;
    case lockIn:
        lockInState();//lock the car,it means the output of motor is 0
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
    if(curr_state == lockIn)
        next_state = standBalance;
}

void parameterAdjustReady(void)
{
    curr_state = next_state;
    if(curr_state == standBalance)
        next_state = parameterAdjust;
}

/*
* if angle beyond (-15, 15) degree, swtich state to emergencyBrake state.
*/
void angleOutOfRange(void)//while the angle is out of range,turn to lockIn state
{
    curr_state = next_state;
    if(curr_state == standBalance || curr_state == advanceCtrl || curr_state == directionCtrl)
    {
        next_state = lockIn;

        // args to slow down motors
        left_value  = (int)angle_ctrl_output - speed_ctrl_output - direction_ctrl_output;
        right_value = (int)angle_ctrl_output - speed_ctrl_output + direction_ctrl_output;
    }
}

void goForward(void)
{
	curr_state = next_state;
	if(curr_state == standBalance)
		next_state = advanceCtrl;
}

void turningCircle(void)
{
	curr_state = next_state;
	if(curr_state == standBalance)
		next_state = directionCtrl;
}
/*
* if bluetooth send char 'l', swtich state to lockIn state.
*/
void lockCar(void)
{
    curr_state = next_state;
    if(curr_state == standBalance || curr_state == advanceCtrl || curr_state == directionCtrl)
    {
        next_state = lockIn;

        // args to slow down motors
        left_value  = (int)angle_ctrl_output - speed_ctrl_output - direction_ctrl_output;
        right_value = (int)angle_ctrl_output - speed_ctrl_output + direction_ctrl_output;
    }

}

/*
* if bluetooth send char 'u', swtich state to emergencyBrake state.
*/
void unlockCar(void)
{
    curr_state = next_state;
    if(curr_state == lockIn)
        next_state = standBalance;
}

/*
* if bluetooth send char 't', swtich state to argsAdjust state.
*/
void stableAdjust(void)//this is a state between all states,the state is standBalanceState,every state turn to another must via it
{
    curr_state = next_state;
    if(curr_state == parameterAdjust || curr_state == advanceCtrl || curr_state == lockIn || curr_state == directionCtrl)
        next_state = standBalance;
}

void standBalanceState(void)
{
    static int i = 0;
    static int counter = 0;

    switch(i)
    {
    case 0:
        readSampleMPU6050();
        break;
    case 1:
        getOriginalAngleGyro();//get the original data of angle and gyroscope
        angleFilter();//combine the data of angle and gyroscope to get a ture angle
        angleCtrlPID();//use the ture angle calculate the output of PWM value
        break;
    case 2:
        sampleSpeed();//calculate the sum of pulses in 10 loop
        speedCtrlPID();
        speedCtrlOutput();
        break;
    case 3:
        directionCtrlPID();
        directionCtrlOutput();
        break;
    case 4:
        motorsOutput();
        if(btCommand == 'l') // lock
        {
            Serial3.println(btCommand);
            btCommand = 0;
            lockInState();//print the pid of angle and speed on the phone
            
        }
        else if(btCommand == 'e') // mean send pid arguments from arduino by bluetooth(on standBalance state)
        {
            Serial3.println(btCommand);
            sendArgsData();//print the pid of angle and speed on the phone
            btCommand = 0;
        }
        else if(btCommand == 'g')
        {
            Serial3.println(btCommand);
            Serial3.println(board_angle);//print the angle of board on the phone
            btCommand = 0;
        }
        // if(++counter == 20)
        // {
        //     counter = 0;
        //     Serial3.println(board_angle); // send car's angle per 200ms
        // }
        break;
    default:
        Serial3.println("standBalanceState error!");
        while(1); //stop here
    }

    if(++i >= 5)
        i = 0;
}

void advanceState(void)
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
        sampleSpeed();//print speed here
        speedCtrlPID();
        speedCtrlOutput();
        break;
    case 3:
        directionCtrlPID();
        directionCtrlOutput();
        break;
    case 4:
        motorsOutput();
        if(btCommand == 'l') // mean  the car to change it's state to lockIn from go forward
        {
            Serial3.println(btCommand);
            lockInState();
            btCommand = 0;
        }
        else if(btCommand == 'b') // mean  the car to change it's state to standBalance from go forward
        {
            Serial3.println(btCommand);
            stableAdjust();
            btCommand = 0;
        }
        else if(btCommand == 'e') // mean send pid arguments from arduino by bluetooth
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
        else if(btCommand == 'v')
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

void directionCtrlState(void)
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
        speedCtrlPID();
        speedCtrlOutput();
        break;
    case 3:
        directionCtrlPID();
        directionCtrlOutput();
        break;
    case 4:
        motorsOutput();
        if(btCommand == 'l') // mean  the car to change it's state to lockIn from turning circle
        {
            Serial3.println(btCommand);
            lockInState();
            btCommand = 0;
        }
        else if(btCommand == 'b') // mean  the car to change it's state to standBalance from turning circle
        {
            Serial3.println(btCommand);
            stableAdjust();
            btCommand = 0;
        }
        else if(btCommand == 'e') // mean send pid arguments from arduino by bluetooth
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
        else if(btCommand == 'v')
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

void lockInState(void)
{
    static int l = 0;
    next_state = lockIn;

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
        // slow down motors gently.
        if(left_value > 0)
            --left_value;
        else if(left_value < 0)
            ++left_value;
        if(right_value > 0)
            --right_value;
        else if(right_value < 0)
            ++right_value;
        motorsOutputAdjust(left_value, right_value);
        break;
    case 4:
        if(btCommand == 'e')
        {
            Serial3.println(btCommand);
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

void parameterAdjustState(void)
{
    int left_value  = (int)angle_ctrl_output - speed_ctrl_output - direction_ctrl_output;
    int right_value = (int)angle_ctrl_output - speed_ctrl_output + direction_ctrl_output;

    // slow down motors gently.
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
        delay(5);
    }

    while(!Serial3.available())
        ;//waiting for next command

    while((btCommand=Serial3.read()) != 'f') // 'f' --> confirm;make sure the value of pid you set is right
    {
        while((btCommand != 'p') && (btCommand != 'i') && (btCommand != 'm') && (btCommand != 'f')) // 'p' --> save angle PID args, 'i' --> save speed PID args, 'v' --> save motor dead value
        {
            Serial3.println(btCommand);
            Serial3.println(" is NOT \'p\' , \'i\' or \'m\'.");
            
            while(!Serial3.available())
                ;
            btCommand = Serial3.read();//because the command is wrong,so you should read serial prot
        }

        // if(btCommand == 't')
        // {
        //     Serial3.println("give it up,please setting the value of pid again");
        //     btCommand = 0;
        //     parameterAdjustState();
            
        // }
        
        Serial3.println(btCommand);
        argsAdjustSaveData();
        getAnglePID();
        getSpeedPID();
        getMotorDeadVal();
        sendArgsData();//make sure the parameter is right
        while(!Serial3.available())
                ;
        //btCommand = Serial3.read();//since define 'btCommand = 0' before,so the value of btCommand is 0 before
                
    }

    Serial3.println(btCommand);    
    btCommand = 0;
    next_state = lockIn;

#ifdef _PRINT_ARGS
    Serial.print("angleP:");
    Serial.println(CarArgs.angleCtrlP);
    Serial.print("angleI:");
    Serial.println(CarArgs.angleCtrlI);
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
