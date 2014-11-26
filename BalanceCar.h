#ifndef     __BLANCECAR__H
#define     __BLANCECAR__H

/***All of the car states defined here***/
typedef enum state{
    standBalance,
    bluetoothCtrl,
    emergencyBrake,
    lockIn,
    argsAdjust,
    // ...more states add here
} States;

States curr_state = standBalance;
States next_state = standBalance;

/***Define all arguments can be modified by bluetooth***/
struct CarArguments{
    float angleCtrlP;
    float angleCtrlD;

    float speedCtrlP;
    float speedCtrlI;

    float motorDeadVal;
    //...more arguments add here
} CarArgs;


/*******Macros for dugug*******/
// #define _PRINT_ANGLE


#define DEBUG_LIGHT_PIN 13

#define PWM_MIN     -252
#define PWM_MAX      252

#define ANGLE_P     10.0
#define ANGLE_D     0.0

// MPU6050 args set up
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

double angle_input, angle_output;
double angle_setpoint = -0.089055;
double kp, ki, kd;
PID angle_pid(&angle_input, &angle_output, &angle_setpoint, kp, ki, kd, DIRECT);

volatile unsigned char btCommand = 0;


#endif

