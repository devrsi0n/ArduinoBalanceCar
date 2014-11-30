#ifndef __STATESMACHINE__H
#define __STATESMACHINE__H

#define EMERGENCY_BRAKE_ANGLE_MIN     -15
#define EMERGENCY_BRAKE_ANGLE_MAX      15

#define LEVEL_ANGLE_MIN     -5
#define LEVEL_ANGLE_MAX      5

// global args
extern volatile char btCommand;
extern volatile float board_angle;
extern volatile float angle_ctrl_output;
extern volatile float speed_ctrl_output;
extern volatile float direction_ctrl_output;

#endif
