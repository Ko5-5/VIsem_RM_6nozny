#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <Arduino.h>
#include <ESP32Servo.h>

#define PWM_MIN 0           // TO BE DEFINED
#define PWM_MAX 255         // TO BE DEFINED

class RobotLeg{
    const uint8_t pwm_servo_1;
    const uint8_t pwm_servo_2;
    uint16_t frequency;
    const uint8_t resolution;

    uint8_t angleValue;

    Servo leg_servo1;
    Servo leg_servo2;

public: 
    RobotLeg(uint8_t _pwm1, uint8_t _pwm2, uint16_t _freq=1000, uint8_t _res=8);
    void init();
};

#endif