/*
 * pid.h
 *
 *  Created on: Feb 20, 2012
 *      Author: aml6195
 */

#ifndef PID_H_
#define PID_H_
#include <pthread.h>
#include <semaphore.h>
#include "iodefs.h"
#include "pipeline.h"


typedef struct{
	pthread_t pid_thread;
	pipeline_dat* input;
	double* setpoint;	//pointer to setpoint value
	double e[3]; 		//error history
	double u[3]; 		//output history
	double pk,ik,dk; 	//PID constants
	double integral;
	pipeline_dat output;
}pid_dat;


void* pid_thread(void* param);
void init_pid(pid_dat* pid,double* setpoint, pipeline_dat* input, double pk, double ik, double dk);
void start_pid(pid_dat* pid);



#endif /* PID_H_ */
