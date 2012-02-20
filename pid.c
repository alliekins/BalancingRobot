/*
 * pid.c
 *
 *  Created on: Feb 20, 2012
 *      Author: aml6195
 */

#include "pid.h"

void init_pid(pid_data* pid,double* setpoint, pipeline_dat* input, double pk, double ik, double dk) {

	pid->setpoint=setpoint;
	pid->input = input;
	pid->output.value=0.0;
	pid->e[0]=0.0;
	pid->e[1]=0.0;
	pid->e[2]=0.0;

	pid->u[0]=0.0;
	pid->u[1]=0.0;
	pid->u[2]=0.0;

	pid->pk = pk;
	pid-> ik = ik;
	pid-> dk = dk;
	sem_init(&pid->output.mutex,0,0);
}

void start_pid(pid_data* pid) {
	pthread_create(&pid->pid_thread, NULL, &pid_thread, (void*)pid);
}

void* pid_thread(void* param){


	pid_data* pid = (pid_data*)param;

	while(1){ //run forever. TODO: figure out exit case?
		sem_wait(&pid->input->mutex); //wait for input to change

		pid->u[2] = pid->u[1];	//propogate hitory forward
		pid->u[1] = pid->u[0];
		pid->u[0] = pid->output.value;

		pid->e[2] = pid->e[1];
		pid->e[1] = pid->e[0];
		pid->e[0] = *pid->setpoint - pid->input->value;


		//update output as per pid equation
		//u(k+1) = u(k) + e(k+1)(pk+ik+dk) - e(k)(pk+2dk) + e(k-1)dk
		//pid->output = pid->u[2] + pid->u[0]*(pid->pk+pid->ik+pid->dk) - pid->e[0]*(pid->pk+2*pid->dk) + pid->e[1]*pid->dk;

		pid->output.value=pid->u[0]+
				pid->pk*(pid->e[0]-pid->e[1])
				+ pid->ik*pid->e[0]
				+ pid->dk*(pid->e[0]-(pid->e[1]/2)+pid->e[2]);



		sem_post(&pid->output.mutex);//notify system that output has changed
	}

}
