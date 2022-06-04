#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <Arduino.h>
#include <ESP32Servo.h>

#define PWM_MIN 500  // in us
#define PWM_MAX 2500 // in us
#define PWM_FQ 333

#define POSITION_MOVE_FRONT 40
#define POSITION_MOVE_CENTER 90
#define POSITION_MOVE_BACK -40
#define POSITION_MOVE_UP 150
#define POSITION_MOVE_DOWN 125

#define DELAY 5

class RobotLeg
{
    const uint8_t pwm_servo_1;
    const uint8_t pwm_servo_2;

    uint8_t angleValue;

    int8_t pos_mov_front = POSITION_MOVE_FRONT;
    uint8_t pos_mov_center = POSITION_MOVE_CENTER;
    int8_t pos_mov_back = POSITION_MOVE_BACK;
    uint8_t pos_mov_up = POSITION_MOVE_UP;
    uint8_t pos_mov_down = POSITION_MOVE_DOWN;

    int last_pos_f; // ostatnia pozycja serwa przod tyl
    int last_pos_u; // ostatnia pozycja serwa gora dol

    bool przycisk; // czy noga dotyka ziemi

    Servo leg_servo1; // serwo do przod tyl
    Servo leg_servo2; // serwo do Sgora dol

public:
    RobotLeg(uint8_t _pwm1, uint8_t _pwm2);
    RobotLeg(uint8_t _pwm1, uint8_t _pwm2, uint8_t _pos_ct);
    RobotLeg(uint8_t _pwm1, uint8_t _pwm2, uint8_t _pos_ft, uint8_t _pos_ct, uint8_t _pos_bk, uint8_t _pos_up, uint8_t _pos_dn);
    void init();
    void halfInit();
    void deinit();
    void horizInit();
    void horizDeInit();
    void moveFront();
    void moveCenter();
    void moveBack();
    void moveUp();
    void moveDown();
};

void test_legs(RobotLeg leg1, RobotLeg leg2, RobotLeg leg3, RobotLeg leg4, RobotLeg leg5, RobotLeg leg6);
void move_robot_front_(RobotLeg leg1, RobotLeg leg2, RobotLeg leg3, RobotLeg leg4, RobotLeg leg5, RobotLeg leg6);

#endif
