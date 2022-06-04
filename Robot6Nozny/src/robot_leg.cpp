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

void RobotLeg::moveFront()
{
    leg_servo2.write(pos_mov_center + pos_mov_front);
    /*
    for (pos = last_pos_f; pos <= POSITION_MOVE_FRONT + last_pos_f; pos++)
    {
        leg_servo1.write(pos);
        delay(DELAY);
    }
    */
    last_pos_f = pos_mov_front;
}

void RobotLeg::moveCenter()
{
    leg_servo2.write(pos_mov_center);
    /*
    for (pos = last_pos_f; pos <= POSITION_MOVE_FRONT + last_pos_f; pos++)
    {
        leg_servo1.write(pos);
        delay(DELAY);
    }
    */
    last_pos_f = pos_mov_center;
}

void RobotLeg::moveBack()
{
    leg_servo2.write(pos_mov_center + pos_mov_back);
    /*
    for (int pos = last_pos_f; pos >= last_pos_f - POSITION_MOVE_FRONT; pos--)
    {
        leg_servo1.write(pos);
        delay(DELAY);
    }
    */
    last_pos_f = pos_mov_back;
}

void RobotLeg::moveUp()
{
    leg_servo2.write(pos_mov_up);
    /*
    for (int pos = last_pos_u; pos < pos_mov_up; pos++)
    {
        leg_servo2.write(pos);
        delay(DELAY);
    }
    */
    last_pos_u = pos_mov_up;
}

void RobotLeg::moveDown()
{
    leg_servo2.write(pos_mov_down);
    /*
    int pos=last_pos_u;
    while (!przycisk) {
        pos--;
        leg_servo2.write(pos);
        delay(DELAY);
    }
    */
    last_pos_u = pos_mov_down;
}

void RobotLeg::calibHoriz()
{
    leg_servo1.write(pos_mov_center);
    last_pos_f = pos_mov_center;
}

/*
void RobotLeg::test_legs(RobotLeg leg1,RobotLeg leg2, RobotLeg leg3,RobotLeg leg4, RobotLeg leg5 , RobotLeg leg6){
leg1.moveUp();  leg1.moveFront();    leg1.moveBack();   leg1.moveDown();
leg2.moveUp();   leg2.moveFront();   leg2.moveBack();    leg2.moveDown();
leg3.moveUp();  leg3.moveFront();   leg3.moveBack();    leg3.moveDown();
leg4.moveUp();  leg4.moveFront();   leg4.moveBack();    leg4.moveDown();
leg5.moveUp();  leg5.moveFront();   leg5.moveBack();    leg5.moveDown();
leg6.moveUp();  leg6.moveFront();   leg6.moveBack();    leg6.moveDown();
}
void RobotLeg::move_robot_front_(RobotLeg leg1,RobotLeg leg2, RobotLeg leg3,RobotLeg leg4, RobotLeg leg5 , RobotLeg leg6){
leg1.moveUp();  leg3.moveUp();  leg5.moveUp();
leg1.moveFront();  leg3.moveFront();  leg5.moveFront();
leg1.moveDown();  leg3.moveDown();  leg5.moveDown();
leg2.moveUp();  leg4.moveUp();  leg6.moveUp();
leg2.moveFront();  leg4.moveFront();  leg6.moveFront();
leg2.moveDown();  leg4.moveDown();  leg6.moveDown();
leg1.moveBack();    leg2.moveBack();    leg3.moveBack();    leg4.moveBack();    leg5.moveBack();    leg6.moveBack();
}*/