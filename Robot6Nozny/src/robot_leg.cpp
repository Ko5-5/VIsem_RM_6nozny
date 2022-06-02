#include "robot_leg.h"

RobotLeg::RobotLeg(uint8_t _pwm1, uint8_t _pwm2)
    : pwm_servo_1(_pwm1),
      pwm_servo_2(_pwm2)
{
}

void RobotLeg::init()
{
    leg_servo1.attach(pwm_servo_1, PWM_MIN, PWM_MAX);
    leg_servo2.attach(pwm_servo_2, PWM_MIN, PWM_MAX);
    leg_servo1.setPeriodHertz(PWM_FQ); // Standard 50hz servo
    leg_servo2.setPeriodHertz(PWM_FQ); // Standard 50hz servo
}

void RobotLeg::moveFront()
{
    int pos = 0;
    for (pos = last_pos_f; pos <= POSITION_MOVE_FRONT + last_pos_f; pos++)
    {
        leg_servo1.write(pos);
        delay(DELAY);
    }
    last_pos_f = pos;
}

void RobotLeg::moveBack()
{
    int pos = 0;
    for (int pos = last_pos_f; pos >= last_pos_f - POSITION_MOVE_FRONT; pos--)
    {
        leg_servo1.write(pos);
        delay(DELAY);
    }
    last_pos_f = pos;
}

void RobotLeg::moveUp()
{
    int pos = 0;
    pos = POSYTION_MOVE_UP;
    leg_servo2.write(pos);
    /*
    for (int pos = last_pos_u; pos < POSYTION_MOVE_UP; pos++)
    {
        leg_servo2.write(pos);
        delay(DELAY);
    }*/
    last_pos_u = pos;
}

void RobotLeg::moveDown()
{
    int pos = 0;
    pos = POSYTION_MOVE_DOWN;
    leg_servo2.write(pos);
    /*
    int pos=last_pos_u;
    while (!przycisk) {
        pos--;
        leg_servo2.write(pos);
        delay(DELAY);
    }
    */
    last_pos_u = pos;
}
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
}