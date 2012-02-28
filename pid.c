/*
 * pid.c
 *
 *  Created on: Feb 20, 2012
 *      Author: aml6195
 */

#include "pid.h"
#include "pipeline.h"

/**
 * initalize a PID controller datastructure.
 *
 * @param setpoint 	- pointer to the setpoint
 * @param input		- pointer to a pipeline datasource
 * @param pk		- proportional gain
 * @param ik		- integral gain
 * @param dk		- derivative gain
 */
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


/**
 * Thread that runs the PID algorithm.
 *
 * Blocks on its input pipeline, until data is ready, then writes to its output pipeline
 *
 */

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


		//proportional error is made up of the current error only.
		double proportional =pid->pk*(pid->e[0]);


		//accumulate the sum of all the errors.
		pid->integral+=pid->e[0];

		//Make sure to trim integral so it doesn't "wind up" overly far.
		if (pid->integral > 1/pid->ik) pid->integral = 1/pid->ik;
		if (pid->integral < -1/pid->ik) pid->integral = -1/pid->ik;

		//Integral error is a sum of all the previous error.
		double integral = pid->ik*pid->integral;

		//derivative error dk*dError/dt
		//3 point derivative. http://www.shodor.org/cserd/Resources/Algorithms/NumericalDifferentiation/
		double derivative =pid->dk*(-3*pid->e[0]+4*pid->e[1]-pid->e[2])/2;

		//Print out the components for help with tuning
		//useful to find which parameter is causing oscillations.
		printf("p:%lf,i:%lf,d:%lf \r\n",proportional,integral,derivative);


		double output =proportional+integral+derivative;

		//clip the output at +-1
		if (output > 1.0) output = 1.0;
		if (output < -1.0) output = -1.0;

		//Add to pipeline. When it is full, it will notify the consumer
		add_to_pipeline(&pid->output, output);
	}

}
