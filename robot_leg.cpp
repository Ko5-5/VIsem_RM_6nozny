#include "robot_leg.h"

RobotLeg::RobotLeg(uint8_t _pwm1, uint8_t _pwm2, uint16_t _freq=1000, uint8_t _res=8)
    :pwm_servo_1(_pwm1),
    pwm_servo_2(_pwm2),
    frequency(_freq),
    resolution(_res)
{}

void RobotLeg::init(){
    leg_servo1.setPeriodHertz(50);      // Standard 50hz servo
	leg_servo2.setPeriodHertz(50);      // Standard 50hz servo
    leg_servo1.attach(pwm_servo_1, PWM_MIN, PWM_MAX);
    leg_servo2.attach(pwm_servo_2, PWM_MIN, PWM_MAX);
}


void RobotLeg::move_f (){
	for (int pos = last_pos_f; pos <= MOVE_F+last_pos_f; pos ++) { 
	    leg_servo1.write(pos);              
	    delay(DELAY);                       
  }
  last_pos_f=pos;
}
void RobotLeg::move_b (){
    for (int pos = last_pos_f; pos >= last_pos_f-MOVE_F; pos --) { 
	    leg_servo1.write(pos);              
	    delay(DELAY);                       
  }	
  last_pos_f=pos;
}
void RobotLeg::move_u (){
    for (int pos = last_pos_u; pos <POSYTION_MOVE_U; pos ++){ 
	    leg_servo2.write(pos);              
	    delay(DELAY);                       
  }
  last_pos_u=pos;
}
void RobotLeg::move_d (){
	int pos=last_pos_u
    while (!przycisk) {
    	pos--;
	    leg_servo2.write(pos);              
	    delay(DELAY);                       
  }
  last_pos_u=pos;
}
