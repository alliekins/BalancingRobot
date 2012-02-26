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

typedef struct{
	double value;
	double values[100];
	int index;
	int size;
	double average;

	sem_t mutex;
}pipeline_dat;



typedef struct{
	pthread_t pid_thread;
	pipeline_dat* input;
	double* setpoint;	//pointer to setpoint value
	double e[3]; 		//error history
	double u[3]; 		//output history
	double pk,ik,dk; 	//PID constants
	pipeline_dat output;

}pid_data;

void* pid_thread(void* param);
void init_pid(pid_data* pid,double* setpoint, pipeline_dat* input, double pk, double ik, double dk);
void start_pid(pid_data* pid);


void add_to_pipeline(pipeline_dat* dat, double val);
void init_pipeline(pipeline_dat* dat, int size);


#endif /* PID_H_ */
