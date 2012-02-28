/*
 * pid.c
 *
 *  Created on: Feb 20, 2012
 *      Author: aml6195
 */

#include "pid.h"
#include "pipeline.h"


void init_pid(pid_dat* pid,double* setpoint, pipeline_dat* input, double pk, double ik, double dk) {

	pid->setpoint=setpoint;
	pid->input = input;
	pid->e[0]=0.0;
	pid->e[1]=0.0;
	pid->e[2]=0.0;

	pid->u[0]=0.0;
	pid->u[1]=0.0;
	pid->u[2]=0.0;

	pid->pk = pk;
	pid-> ik = ik;
	pid-> dk = dk;

	pid->integral=0;

	init_pipeline(&pid->output, 2);

}

void start_pid(pid_dat* pid) {
	pthread_create(&pid->pid_thread, NULL, &pid_thread, (void*)pid);
}

void* pid_thread(void* param){


	pid_dat* pid = (pid_dat*)param;

	while(1){ //run forever. TODO: figure out exit case?
		sem_wait(&pid->input->mutex); //wait for input to change

		pid->u[2] = pid->u[1];	//propogate hitory forward
		pid->u[1] = pid->u[0];
		pid->u[0] = pid->output.lastInput;

		pid->e[2] = pid->e[1];
		pid->e[1] = pid->e[0];
		pid->e[0] = (*pid->setpoint - pipe_get_average(pid->input));


		//printf("error: %lf average:%lf\r\n",pid->e[0], pipe_get_average(pid->input));

		//update output as per pid equation
		//u(k+1) = u(k) + e(k+1)(pk+ik+dk) - e(k)(pk+2dk) + e(k-1)dk
		//pid->output.value = pid->u[2] + pid->u[0]*(pid->pk+pid->ik+pid->dk) - pid->e[0]*(pid->pk+2*pid->dk) + pid->e[1]*pid->dk;
		//double proportional =pid->pk*(pid->e[0]-pid->e[1]);
		//double derivative =pid->dk*(pid->e[0]-(pid->e[1]/2)+pid->e[2]);
		//double integral = pid->u[0]+pid->ik*pid->e[0];

		double proportional =pid->pk*(pid->e[0]);

		pid->integral+=pid->e[0];
		if (pid->integral > 1/pid->ik) pid->integral = 1/pid->ik;
		if (pid->integral < -1/pid->ik) pid->integral = -1/pid->ik;

		double integral = pid->ik*pid->integral;
		double derivative =pid->dk*(-3*pid->e[0]+4*pid->e[1]-pid->e[2])/2;


		printf("p:%lf,i:%lf,d:%lf \r\n",proportional,integral,derivative);

		double output =proportional+integral+derivative;
		if (output > 1.0) output = 1.0;
		if (output < -1.0) output = -1.0;


		add_to_pipeline(&pid->output, output);

		//sem_post(&pid->output.mutex);//notify system that output has changed
	}

}
