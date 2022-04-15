#ifndef PINOUT_H
#define PINOUT_H

#include <Arduino.h>

// I2C pins
#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_INT     23

// Proximity sensors
#define PROX_TRIG_1 19
#define PROX_TRIG_2 18
#define PROX_ECHO_1 34
#define PROX_ECHO_2 35

// Voltage measure
#define VOL_MEASURE 39

// Servos pwm
#define PWM_1       32
#define PWM_2       33
#define PWM_3       25
#define PWM_4       26
#define PWM_5       27
#define PWM_6       14
#define PWM_7       5
#define PWM_8       17
#define PWM_9       16
#define PWM_10      4
#define PWM_11      15
#define PWM_12      13

void pinout_init();

#endif