#ifndef __ANGLE__H
#define __ANGLE__H

extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

volatile float original_angle = 0;
volatile float original_gyro  = 0;
volatile float board_angle = 0;

volatile float angle_ctrl_output = 0;

#define MAX_ANGLE   25
#define MIN_ANGLE   -25

#define GRY_OFFSET  296
#define GYR_GAIN    0.00763
#define ACC_GAIN    0.000061

#define PI          3.14159326


#endif

