/*
 * pwm.h
 *
 *  Created on: Feb 24, 2012
 *      Author: aah9038
 */

#ifndef PWM_H_
#define PMW_H_

typedef struct{
	int high_time;
	int period;

	char currentOutput;
	uintptr_t cnt_port;	//output port
	char output_mask; //mask to determine what pins to write to
	char stop;

	pthread_t output_thread;

}pwm_args;

void pwm_setDuty(pwm_args* args, double duty);
void* pwm_thread(void* param);
void pwm_setDuty(pwm_args* args, double duty);
void startPWM(pwm_args* args,uintptr_t outputPort, char output_mask, int high_time, int period);

#endif /* PWM_H_ */
