#include "Motors.h"

void initMotors(void)
{
    setPWMFrequency(HZ_31250);

    pinMode(LEFT_PWM_PIN, OUTPUT);
    pinMode(LEFT_IN1_PIN, OUTPUT);
    pinMode(LEFT_IN2_PIN, OUTPUT);

    pinMode(RIGHT_PWM_PIN, OUTPUT);
    pinMode(RIGHT_IN1_PIN, OUTPUT);
    pinMode(RIGHT_IN2_PIN, OUTPUT);


    digitalWrite(LEFT_PWM_PIN, LOW);
    digitalWrite(RIGHT_PWM_PIN, LOW);

    digitalWrite(LEFT_IN1_PIN, HIGH);
    digitalWrite(LEFT_IN2_PIN, LOW);
    digitalWrite(RIGHT_IN1_PIN, HIGH);
    digitalWrite(RIGHT_IN2_PIN, LOW);
}

void initEncoders(void)
{
    pinMode(LEFT_ENCODER_PIN,  INPUT);
    pinMode(RIGHT_ENCODER_PIN, INPUT);

    pinMode(LEFT_DIRECTION_PIN, INPUT);
    pinMode(RIGHT_DIRECTION_PIN, INPUT);

    attachInterrupt(4, encoderLeft,  FALLING); // open external interrupt #4
    attachInterrupt(5, encoderRight, FALLING);
}

void sampleSpeed(void)
{
    static int time_counter = 0;
    static unsigned long past_micros = 0;

    if(time_counter++ == SAMPLE_SPEED_PERIOD)
    {
        time_counter = 0;

        unsigned long curr_micros = micros();
        float dt = (float)(curr_micros - past_micros);
        past_micros = curr_micros;

        detachInterrupt(4); // close external interrupt #4
        detachInterrupt(5); // close external interrupt #5

        rpm_left  = (float)count_left  * 60.0 * (1000000.0 / dt) / (400.0 * 32.0);
        rpm_right = (float)count_right * 60.0 * (1000000.0 / dt) / (400.0 * 32.0);
        count_left  = 0;
        count_right = 0;
        attachInterrupt(4, encoderLeft,  FALLING); // reopen interrupt
        attachInterrupt(5, encoderRight, FALLING);
    }

#ifdef _PRINT_SPEED
    Serial.print("Speed:");
    Serial.print(rpm_left);
    Serial.print(",");
    Serial.println(rpm_right);
#endif

}

/*
* external interrupt service routine for encoder
*/
void encoderLeft(void)
{
    // if the motor is forewarding, it's unnecessary to read direction pin.
    if (rpm_left > 10.0)
    {
        count_left++;
        return;
    }
    if (rpm_left < -10.0)
    {
        count_left--;
        return;
    }
    // the motor is forewarding if B line is high level voltage when a falling extern interruption is coming,
    // otherwise the motor is rollbacking.
    if (digitalRead(LEFT_DIRECTION_PIN) == HIGH)
        count_left++;
    else
        count_left--;
}
void encoderRight(void)
{
    if (rpm_right > 10.0)
    {
        count_right++;
        return;
    }
    if (rpm_left < -10.0)
    {
        count_right--;
        return;
    }

    if (digitalRead(RIGHT_DIRECTION_PIN) == HIGH)
        count_right++;
    else
        count_right--;
}

/*
* caculate speed PID output
*/
void speedCtrl(void)
{
    static int time_counter = 0;
    if(time_counter++ == SPEED_CTRL_PERIOD)
    {
        time_counter = 0;
        if(next_state == bluetoothCtrl && btCommand == 'w')
        {
            Serial3.println(btCommand); // let operator knows whether the bluetooth command write into controller board.
            set_car_speed += 20;
            sendCarSpeed();
            btCommand = 0;
        }
        else if(next_state == bluetoothCtrl && btCommand == 's')
        {
            Serial3.println(btCommand);
            set_car_speed -= 20;
            sendCarSpeed();
            btCommand = 0;
        }
        set_car_speed = constrain(set_car_speed, SPEED_LIMIT_MIN, SPEED_LIMIT_MAX);
        float average_speed = (rpm_left + rpm_right) / 2.0;
        speed_ctrl_total_output = computeSpeedPID(set_car_speed, average_speed);
    }
}

/*
* speed PID output divide equally
*/
void speedCtrlOutput(void)
{
    static int counter = 0;
    static float speed_output_old = 0;
    float delta_speed_output = 0;
    float speed_output_new = speed_ctrl_total_output;

    delta_speed_output =  speed_output_new - speed_output_old;
    speed_ctrl_output = counter * delta_speed_output / SPEED_CTRL_PERIOD + speed_output_old;
    if(counter++ == SPEED_CTRL_PERIOD)
    {
        counter = 0;
        speed_output_old = speed_output_new;
    }
}

/*
* caculate direction output to 10 parts
*/
void directionCtrl(void)
{
    static int time_counter = 0;

    if(time_counter++ == DIRECTION_CTRL_PERIOD)
    {
        time_counter = 0;
        direction_ctrl_total_output = 0;
        if(next_state == bluetoothCtrl && btCommand == 'a')
        {
            Serial3.println(btCommand);
            direction_ctrl_total_output = 30;
            btCommand = 0;
        }
        else if(next_state == bluetoothCtrl && btCommand == 'd')
        {
            Serial3.println(btCommand);
            direction_ctrl_total_output = -30;
            btCommand = 0;
        }
    }
}

/*
* direction output divide equally to 10 parts
*/
void directionCtrlOutput(void)
{
    static int counter = 0;
    static float direction_output_old = 0;
    float direction_output_new = direction_ctrl_total_output;
    float delta_direction_output = direction_output_new - direction_output_old;

    direction_ctrl_output = delta_direction_output * counter / DIRECTION_CTRL_PERIOD + direction_output_old;
    if(counter++ == DIRECTION_CTRL_PERIOD)
    {
        counter = 0;
        direction_output_old = direction_output_new;
    }
}

void motorsOutput(void)
{
    float left_value  = angle_ctrl_output - speed_ctrl_output - direction_ctrl_output;
    float right_value = angle_ctrl_output - speed_ctrl_output + direction_ctrl_output;

    motorsOutputAdjust(left_value, right_value);
}

/*
* add motor dead value and limit motor output
*/
void motorsOutputAdjust(float left_value, float right_value)
{
    if(left_value >= 0)
        left_value  += CarArgs.motorDeadVal;
    else
        left_value  -= CarArgs.motorDeadVal;
    if(right_value >= 0)
        right_value += CarArgs.motorDeadVal;
    else
        right_value -= CarArgs.motorDeadVal;
    left_value  = constrain(left_value,  PWM_MIN, PWM_MAX);
    right_value = constrain(right_value, PWM_MIN, PWM_MAX);

    setMotorsVoltage(left_value, right_value);
}

/*
* control motor speed and direction by pwm value
*/
void setMotorsVoltage(float pwm_left, float pwm_right)
{
    int pwm_output_left  = (int)round(pwm_left);
    int pwm_output_right = (int)round(pwm_right);

    if(pwm_output_left < 0)
    {
        pwm_output_left = -pwm_output_left;
        digitalWrite(LEFT_IN1_PIN, HIGH);
        digitalWrite(LEFT_IN2_PIN, LOW);
    }
    else
    {
        digitalWrite(LEFT_IN1_PIN, LOW);
        digitalWrite(LEFT_IN2_PIN, HIGH);
    }
    analogWrite(LEFT_PWM_PIN, pwm_output_left);


    if(pwm_output_right < 0)
    {
        pwm_output_right = -pwm_output_right;
        digitalWrite(RIGHT_IN1_PIN, HIGH);
        digitalWrite(RIGHT_IN2_PIN, LOW);
    }
    else
    {
        digitalWrite(RIGHT_IN1_PIN, LOW);
        digitalWrite(RIGHT_IN2_PIN, HIGH);
    }
    analogWrite(RIGHT_PWM_PIN, pwm_output_right);
}


void setPWMFrequency(byte mode)
{
    if(mode >= 1 && mode <= 5)
    {
        // set PWM frequency for pin 2, 3, 5
        // TCCR3B = TCC3RB & 0b11111000 | mode;
        // set PWM frequency for pin 6, 7, 8
        // TCCR4B = TCCR4B & 0b11111000 | mode;

        // set PWM frequency for pin 9, 10
        TCCR2B = TCCR2B & 0b11111000 | mode;
    }
}

float computeSpeedPID(float set_speed, float average_speed)
{
    static float intergral = 0;
    float result = 0;
    float delta_speed = (set_speed - average_speed);
    float fP = delta_speed * CarArgs.speedCtrlP;
    float fI = delta_speed * CarArgs.speedCtrlI;

    intergral += fI;
    intergral = constrain(intergral, INTERGRAL_MIN, INTERGRAL_MAX);
    result = fP + intergral;

    return result;
}


