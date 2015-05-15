#ifndef __ANGLE__H
#define __ANGLE__H

#include <MPU6050.h>

#define MAX_ANGLE   25
#define MIN_ANGLE   -25

// global args
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

// args for angle filter
volatile float original_angle = 0;
volatile float original_gyro  = 0;
volatile float klm_angle = 0;
volatile float board_angle = 0;
volatile float board_gyro  = 0;

// args for angle control
volatile float angle_ctrl_output = 0;

#endif

