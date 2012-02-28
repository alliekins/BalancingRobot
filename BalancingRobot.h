/*
 * BalancingRobot.h
 *
 *  Created on: Feb 27, 2012
 *      Author: aml6195
 */

#ifndef BALANCINGROBOT_H_
#define BALANCINGROBOT_H_

#include "motor.h"
#include "pid.h"
#include "accelerometer.h"

typedef struct{
	motor_t 	motorA;
	motor_t 	motorB;

	accel_dat 	accelerometer;
	pid_dat		PID;

	double 		setpoint;

}robot_data;

#endif /* BALANCINGROBOT_H_ */
