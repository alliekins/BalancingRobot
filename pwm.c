/*
 * pwm.c
 *
 *  Created on: Feb 24, 2012
 *      Author: aah9038
 */

#include "pwm.h"
#include <sched.h>

/**
 * PWM thread.
 *
 * This thread simply runs until the PWM structure is asked to stop.
 *
 */
void* pwm_thread(void* param){
	pwm_args* args = (pwm_args*)param;

	sem_wait(&port_mutex);
	args->currentOutput= in8(args->cnt_port);
	out8(args->cnt_port,args->currentOutput | (args->output_mask)); //set high
	sem_post(&port_mutex);

	struct itimerspec value;

	while(!args->stop){

		//set the pin high
		if(args->high_time>0){
			sem_wait(&port_mutex);
			args->currentOutput= in8(args->cnt_port);
			out8(args->cnt_port,args->currentOutput | (args->output_mask)); //set high
			sem_post(&port_mutex);
			value.it_value.tv_nsec = args->high_time*50000;
			value.it_value.tv_sec = 0;
			nanosleep(&value,NULL); //more friendly
			//nanospin(&value);//much faster.. up it_value
		}

		//set the pin low
		if(args->period-args->high_time>0){
			sem_wait(&port_mutex);
			args->currentOutput= in8(args->cnt_port);
			out8(args->cnt_port,args->currentOutput & ~(args->output_mask)); //set low
			sem_post(&port_mutex);

			value.it_value.tv_nsec = (args->period-args->high_time)*50000;
			value.it_value.tv_sec = 0;
			nanosleep(&value,NULL); //more friendly
			//nanospin(&value);//much faster.. up it_value
		}
	}

	//set the output to 0 before leaving
	out8(args->cnt_port,args->currentOutput & ~(args->output_mask)); //set all the controlled pins to low

	return 0;
}


/**
 * Set the PWM duty cycle
 *
 *
 */
void pwm_setDuty(pwm_args* args, double duty){

	args->high_time=(int) (duty * args->period);

	//clip high time to 0 and period
	args->high_time = args->high_time < 0? 0 :args->high_time;
	args->high_time = args->high_time > args->period ? args->period : args->high_time;

}


/**
 * Initialize and start the PWM thread
 *
 */
void startPWM(pwm_args* args,uintptr_t outputPort, char output_mask, int high_time, int period){

	args->high_time=high_time;
	args->period=period;
	args->cnt_port=outputPort;
	args->output_mask=output_mask;
	args->currentOutput=0;
	args->stop=0;

	pthread_create(&args->output_thread,NULL, &pwm_thread,(void*)args); //create a new thread for pwm
	//set the priority to max so we get decent PWM. This beats the crap out of the CPU
	pthread_setschedprio(args->output_thread,sched_get_priority_max(SCHED_FIFO));

}
