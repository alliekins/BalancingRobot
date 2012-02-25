/*
 * motor.h
 *
 *  Created on: Feb 24, 2012
 *      Author: aah9038
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdint.h>
#include "pwm.h"

typedef enum {
	freewheel	= 0b00,
			forward 	= 0b01,
			backward	= 0b10,
			brake		= 0b11
}motor_mode;

/**
 * L298N motor controller driver
 *
 *
 */
typedef struct{
	uintptr_t cnt_port; //motor control ouput port
	char input_1_pin;
	char input_2_pin;
	motor_mode current_mode;
	pwm_args pwm;
}motor_t;

void motor_setSpeed(motor_t* motor, double speed);
void motor_setMode(motor_t* motor, motor_mode mode);
void init_motor(motor_t* motor, uintptr_t port, char input_1_pin, char input_2_pin, char pwmPin);

#endif /* MOTOR_H_ */
