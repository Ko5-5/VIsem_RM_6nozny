#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <Arduino.h>
#include <ESP32Servo.h>

#define PWM_MIN 0           // TO BE DEFINED
#define PWM_MAX 255         // TO BE DEFINED
#define MOVE_F 20
#define POSYTION_MOVE_U 40
#define DELAY 15
class RobotLeg{
    const uint8_t pwm_servo_1;
    const uint8_t pwm_servo_2;
    uint16_t frequency;
    const uint8_t resolution;

    uint8_t angleValue;
    
	int last_pos_f;			//ostatnia pozycja serwa przod tyl
	int last_pos_u;			//ostatnia pozycja serwa gora dol
	
	bool przycisk;           // czy noga dotyka ziemi 
	
    Servo leg_servo1;       //serwo do przod tyl
    Servo leg_servo2;		//serwo do Sgora dol

public: 
    RobotLeg(uint8_t _pwm1, uint8_t _pwm2, uint16_t _freq=1000, uint8_t _res=8);
    void init();
    void move_f ();
    void move_b ();
    void move_u ();
    void move_d ();
};

#endif
