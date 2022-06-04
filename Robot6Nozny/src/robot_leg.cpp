#include "robot_leg.h"

RobotLeg::RobotLeg(uint8_t _pwm1, uint8_t _pwm2)
    : pwm_servo_1(_pwm1),
      pwm_servo_2(_pwm2)
{
}

RobotLeg::RobotLeg(uint8_t _pwm1, uint8_t _pwm2, uint8_t _pos_ct)
    : pwm_servo_1(_pwm1),
      pwm_servo_2(_pwm2),
      pos_mov_center(_pos_ct)
{
}

RobotLeg::RobotLeg(uint8_t _pwm1, uint8_t _pwm2, uint8_t _pos_ft, uint8_t _pos_ct, uint8_t _pos_bk, uint8_t _pos_up, uint8_t _pos_dn)
    : pwm_servo_1(_pwm1),
      pwm_servo_2(_pwm2),
      pos_mov_front(_pos_ft),
      pos_mov_center(_pos_ct),
      pos_mov_back(_pos_bk),
      pos_mov_up(_pos_up),
      pos_mov_down(_pos_dn)
{
}

void RobotLeg::init()
{
    leg_servo1.attach(pwm_servo_1, PWM_MIN, PWM_MAX);
    leg_servo2.attach(pwm_servo_2, PWM_MIN, PWM_MAX);
    leg_servo1.setPeriodHertz(PWM_FQ); // Standard 50hz servo
    leg_servo2.setPeriodHertz(PWM_FQ); // Standard 50hz servo
}

void RobotLeg::halfInit()
{
    leg_servo2.attach(pwm_servo_2, PWM_MIN, PWM_MAX);
    leg_servo2.setPeriodHertz(PWM_FQ); // Standard 50hz servo
}

void RobotLeg::deinit()
{
    leg_servo1.detach();
    leg_servo2.detach();
}

void RobotLeg::horizInit()
{
    leg_servo1.attach(pwm_servo_1, PWM_MIN, PWM_MAX);
    leg_servo1.setPeriodHertz(PWM_FQ); // Standard 50hz servo
}
void RobotLeg::horizDeInit()
{
    leg_servo1.detach();
    leg_servo1.pwm.detachPin(pwm_servo_1);
}

void RobotLeg::moveFront()
{
    leg_servo1.write(pos_mov_center + pos_mov_front);
    last_pos_f = pos_mov_front;
}

void RobotLeg::moveCenter()
{
    leg_servo1.write(pos_mov_center);
    /*
    if (last_pos_f > pos_mov_center)
    {
        for (int pos = last_pos_f; pos >= pos_mov_center; pos--)
        {
            leg_servo1.write(pos);
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
    else
    {
        for (int pos = last_pos_f; pos <= pos_mov_center; pos++)
        {
            leg_servo1.write(pos);
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
    */
    last_pos_f = pos_mov_center;
}

void RobotLeg::moveBack()
{
    leg_servo1.write(pos_mov_center + pos_mov_back);
    last_pos_f = pos_mov_back;
}

void RobotLeg::moveUp()
{
    leg_servo2.write(pos_mov_up);
    last_pos_u = pos_mov_up;
}

void RobotLeg::moveDown()
{
    leg_servo2.write(pos_mov_down);
    last_pos_u = pos_mov_down;
}