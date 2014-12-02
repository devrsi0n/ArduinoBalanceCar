#ifndef _MY_EEPROM_H
#define _MY_EEPROM_H

#include <EEPROM.h>

// EEPROM address for argments adjust
#define EEPROM_ALREADY_INITIALIZE_ADDR          0
#define EEPROM_ANGLE_P_ADDR                     2
#define EEPROM_ANGLE_I_ADDR                     4
#define EEPROM_ANGLE_D_ADDR                     6
#define EEPROM_SPEED_P_ADDR                     8
#define EEPROM_SPEED_I_ADDR                     10
#define EEPROM_SPEED_D_ADDR                     12
#define EEPROM_ANGLE_PD_SAVED_ADDR              14
#define EEPROM_SPEED_PI_SAVED_ADDR              16
#define EEPROM_MOTOR_DEAD_VAL_ADDR              18
#define EEPROM_MOTOR_DEAD_VAL_SAVED_ADDR        20

// flags for argments adjust
#define ANGLE_PD_SAVED                          10
#define SPEED_PI_SAVED                          10
#define ALREADY_INITIALIZE                      10
#define MOTOR_DEAD_VAL_SAVED                    10


#endif
