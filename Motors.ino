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
    pinMode(LEFT_ENCODER_PIN, OUTPUT);
    pinMode(RIGHT_ENCODER_PIN, OUTPUT);

    attachInterrupt(4, encoderLeft,  FALLING); // open external interrupt #4
    attachInterrupt(5, encoderRight, FALLING);
}

void sampleSpeed(void)
{
    static int time_counter = 0;
    static unsigned long past_micros = 0;
    unsigned long curr_micros = micros();
    float dt = (curr_micros - past_micros) / 1000000.0;
    past_micros = curr_micros;

    if(time_counter++ == 20)
    {
        time_counter = 0;

        detachInterrupt(4); // close external interrupt #4
        detachInterrupt(5); // close external interrupt #5

        rpm_left  = count_left  * 60.0 * (1000.0 / dt) / (400.0 * 32.0);
        rpm_right = count_right * 60.0 * (1000.0 / dt) / (400.0 * 32.0);
        count_left  = 0;
        count_right = 0;
        attachInterrupt(4, encoderLeft,  FALLING); // reopen interrupt
        attachInterrupt(5, encoderRight, FALLING);
    }

#ifdef _PRINT_SPEED
    Serial.print("Speed:")
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
    count_left++;
}
void encoderRight(void)
{
    count_right++;
}

void speedCtrl(void)
{

}

void speedCtrlOutput(void)
{

}

void directionCtrl(void)
{

}

void directionCtrlOutput(void)
{

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

