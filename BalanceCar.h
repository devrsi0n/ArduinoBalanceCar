#ifndef     __BALANCECAR__H
#define     __BALANCECAR__H

/***All of the car's states defined here***/
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
    float angleCtrlI;
    float angleCtrlD;

    float speedCtrlP;
    float speedCtrlI;
    float speedCtrlD;

    float motorDeadVal;
    //...more arguments add here
} CarArgs;


/*******Macros for dugug*******/
// #define _PRINT_ANGLE
// #define _PRINT_ARGS
// #define _PRINT_SPEED


#define RUNNING_LIGHT_PIN 13

#define PWM_MIN     -252
#define PWM_MAX      252

// static args value
#define ANGLE_P     13.0
#define ANGLE_I     0
#define ANGLE_D     0.02

#define SPEED_P     0
#define SPEED_I     0
#define SPEED_D     0

#define MOTOR_OUT_DEAD_VAL 0

// set up MPU6050 args
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// set up angle PID args
double angle_input = 0, angle_output = 0;
double angle_setpoint = 0.326865; //-1.777692; //-0.089055;
double kp, ki, kd;
PID angle_pid(&angle_input, &angle_output, &angle_setpoint, kp, ki, kd, DIRECT);

// bluetooth command for argments adjust and control
volatile char btCommand = 0;

// set car's speed for speed PID control
volatile int set_car_speed = 0;

// global args for angle filter
extern volatile float original_angle;
extern volatile float klm_angle;
extern volatile float board_angle;

extern volatile float original_gyro;
extern volatile float board_gyro;

#endif

