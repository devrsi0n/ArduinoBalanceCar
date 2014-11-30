#ifndef     __MOTORS_H
#define     __MOTORS_H

#include <math.h>

// define all of the motor pins below
#define LEFT_PWM_PIN    9
#define LEFT_IN1_PIN    4
#define LEFT_IN2_PIN    5

#define RIGHT_PWM_PIN   10
#define RIGHT_IN1_PIN   6
#define RIGHT_IN2_PIN   7

// define encoder pins for measure car's speed
#define LEFT_ENCODER_PIN    19
#define RIGHT_ENCODER_PIN   18


// PWM frequency selector
#define HZ_31250        1
#define HZ_3906         2
#define HZ_488          3
#define HZ_122          4
#define HZ_30           5

// limit speed & intergral max value
#define SPEED_LIMIT_MIN     -60
#define SPEED_LIMIT_MAX      60
#define INTERGRAL_MIN       -25
#define INTERGRAL_MAX        25

// control period
#define SPEED_CTRL_PERIOD        20
#define DIRECTION_CTRL_PERIOD    10

// args for speed sample
volatile float rpm_left = 0; // car's speed (rotations per minute)
volatile float rpm_right = 0;
volatile long count_left  = 0; // counter for left encoder
volatile long count_right = 0;

// args for speed and direction output
volatile float speed_ctrl_total_output = 0;
volatile float direction_ctrl_total_output = 0;
volatile float speed_ctrl_output = 0;
volatile float direction_ctrl_output = 0;
extern volatile int set_car_speed;

#endif





