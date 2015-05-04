#include "PID.h"

/*
* caculate angle's PID output
*/
float anglePIDCompute(float angle)
{
    float result = 0;
    static float integral = 0;
    static float last_angle = 0;
    float error = ANGLE_SETPIONT - angle;
    float diff_error = angle - last_angle;
    last_angle = angle;

    integral += CarArgs.angleCtrlI * error;
    integral = constrain(integral, ANGLE_INTEGRAl_MIN, ANGLE_INTEGRAL_MAX); // limit integral item
    result = CarArgs.angleCtrlP * error + integral - CarArgs.angleCtrlD * diff_error;
    result = constrain(result, PWM_MIN, PWM_MAX);

    return result;
}

/*
* caculate speed's PID output
*/
float speedPIDcompute(float set_speed, float average_speed)
{
    float result = 0;
    static float intergral = 0;
    static float last_speed = 0;
    float error = set_speed - average_speed;
    float diff_error = average_speed - last_speed;
    last_speed = average_speed;

    intergral += error * CarArgs.speedCtrlI;
    intergral = constrain(intergral, SPEED_INTERGRAL_MIN, SPEED_INTERGRAL_MAX);
    result = error * CarArgs.speedCtrlP + intergral - diff_error * CarArgs.speedCtrlD;
    result = constrain(result, PWM_MIN, PWM_MAX);

    return result;
}



