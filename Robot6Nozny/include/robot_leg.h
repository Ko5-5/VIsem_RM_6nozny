#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <Arduino.h>
#include <ESP32Servo.h>

#define PWM_MIN 500  // in us
#define PWM_MAX 2500 // in us
#define PWM_FQ 333

#define POSITION_MOVE_FRONT 20
#define POSYTION_MOVE_UP 180
#define POSYTION_MOVE_DOWN 135
#define DELAY 5

class RobotLeg
{
public:
    const uint8_t pwm_servo_1;
    const uint8_t pwm_servo_2;

    uint8_t angleValue;

    int last_pos_f; // ostatnia pozycja serwa przod tyl
    int last_pos_u; // ostatnia pozycja serwa gora dol

    bool przycisk; // czy noga dotyka ziemi

    Servo leg_servo1; // serwo do przod tyl
    Servo leg_servo2; // serwo do Sgora dol

public:
    RobotLeg(uint8_t _pwm1, uint8_t _pwm2);
    void init();
    void moveFront();
    void moveBack();
    void moveUp();
    void moveDown();
};

void test_legs(RobotLeg leg1,RobotLeg leg2, RobotLeg leg3,RobotLeg leg4, RobotLeg leg5 , RobotLeg leg6);
void move_robot_front_(RobotLeg leg1,RobotLeg leg2, RobotLeg leg3,RobotLeg leg4, RobotLeg leg5 , RobotLeg leg6);


#endif
