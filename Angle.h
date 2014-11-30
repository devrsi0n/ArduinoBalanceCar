#ifndef __ANGLE__H
#define __ANGLE__H

#define MAX_ANGLE   25
#define MIN_ANGLE   -25

#define PI          3.14159326

extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

volatile float original_angle = 0;
volatile float original_gyro  = 0;
volatile float board_angle = 0;

volatile float angle_ctrl_output = 0;

#endif

