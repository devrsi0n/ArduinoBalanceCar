#ifndef __STATESMACHINE__H
#define __STATESMACHINE__H

// boundary condition
#define EMERGENCY_BRAKE_ANGLE_MIN     -15
#define EMERGENCY_BRAKE_ANGLE_MAX      15

#define LEVEL_ANGLE_MIN     -5
#define LEVEL_ANGLE_MAX      5

#define _PRINT_ARGS

// PWM left & right value to slow down motors
volatile int left_value  = 0;
volatile int right_value = 0;

// global args
extern volatile char btCommand;
extern volatile float board_angle;
extern volatile float angle_ctrl_output;
extern volatile float speed_ctrl_output;
extern volatile float direction_ctrl_output;

#endif
