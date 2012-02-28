/*
 * motor.c
 *
 *  Created on: Feb 24, 2012
 *      Author: aah9038
 */

#include "motor.h"

/**
 * Set the motor speed
 *
 * Accepts a signed double from -1 to 1 that determines speed of the motor
 */
void motor_setSpeed(motor_t* motor, double speed){

	if(speed <0.0){
		motor_setMode(motor, backward);
	}
	else{
		motor_setMode(motor, forward);
	}

	//Dead zone insertion
	if(fabs(speed)>0.01){
		pwm_setDuty(&motor->pwm, fabs(speed));
	}else{

		pwm_setDuty(&motor->pwm, 0);
	}
}


/**
 * Set the motor mode
 *
 * mode=freewheel,forward,backward,brake
 */
void motor_setMode(motor_t* motor, motor_mode mode){

	if (motor->current_mode!=mode){
		motor->current_mode=mode;

		//set the appropraite bits of  the motor control.
		setPin(motor->cnt_port,motor->input_1_pin,(mode & 0b01));
		setPin(motor->cnt_port,motor->input_2_pin,(mode & 0b10));
	}
}

/**
 * Initializes a motor object
 *
 * mode=freewheel,forward,backward,brake
 */
void init_motor(motor_t* motor, uintptr_t port, char input_1_pin, char input_2_pin, char pwmPin){

	motor->cnt_port=port;
	motor->input_1_pin=input_1_pin;
	motor->input_2_pin=input_2_pin;

	motor_setMode(motor,freewheel);

	startPWM(&motor->pwm,port,0x1<<pwmPin,0,255);//pwm out on pin A0

}




