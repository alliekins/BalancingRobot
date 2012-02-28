//#include <cstdlib>
//#include <iostream.h>

#include <time.h>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/netmgr.h>
#include <sys/neutrino.h>
#include <semaphore.h>
#include <math.h>
#include <string.h>

#include "iodefs.h"
#include "pid.h"
#include "motor.h"
#include "accelerometer.h"
//#include "encoder.h"		//removed because we didn't have enough time to use it
#include "pipeline.h"
#include "BalancingRobot.h"

void * Control(void* arg);
void * Output(void* arg);
void * Input(void* arg);
void * UserInput (void* arg);
void RunControlAlgorithm();



/**
 * Input thread that allows user to tune PID on the fly.
 *
 * Note: The log file will not change when you change the PID parameters
 */
void * Input(void* arg){
	robot_data* bot = (robot_data*)arg;
	double* ptr;
	double val;

	char buf[50];



	while (1) {
		int i;
		for (i=0;i<50 && (buf[i]=getchar())!='\n';i++);

		if(memcmp(buf,"stop",sizeof("stop"))){
			bot->motorA.pwm.stop=1;
			bot->motorB.pwm.stop=1;
			printf("stopped");
		}
		else{

			char variable = buf[0];
			char operator = buf[1];
			sscanf(buf+2,"%lf", &val);
			switch(variable){
			case 'p':
				ptr = &(bot->PID.pk);
				break;
			case 'i':
				ptr = &(bot->PID.ik);
				break;
			case 'd':
				ptr = &(bot->PID.dk);
				break;
			case 's':
				ptr = bot->PID.setpoint;
				break;
			default:
				printf("Invalid variable\n");
				continue;
			}

			switch(operator) {
			case '=':
				*ptr = val;
				break;
			case '+':
				*ptr += val;
				break;
			case '-':
				*ptr -= val;
				break;

			default:
				printf("Invalid operator\n");
				continue;
			}
			printf("%c:%lf\r\n", variable,*ptr);
		}
	}
}

/**
 * Simple output thread that consumes the PID output values
 *
 */

void * Output(void* arg){
	robot_data* bot = (robot_data*) arg;
	for(;;){
		//consume the PID output
		sem_wait(&bot->PID.output.mutex);
		motor_setSpeed(&bot->motorA,pipe_get_average(&bot->PID.output));
		motor_setSpeed(&bot->motorB,pipe_get_average(&bot->PID.output));
	}
}



int main(int argc, char *argv[]) {


	//get root permissions
	int privity_err;

	privity_err = ThreadCtl( _NTO_TCTL_IO, NULL );
	if ( privity_err == -1)
	{
		perror( "Can't get root permissions\n");
		return -1;
	} else {
	}

	initialize_handles();


	//reset peripheral board
	sem_wait(&port_mutex);
	out8(cnt_base+CMD_REGISTER,RST_BRD);
	out8(cnt_base, RST_DAC); //reset DAC
	sem_post(&port_mutex);

#define CHAN_A_MASK 0x02
#define CHAN_B_MASK 0x04

	//setup port B as input (for encoder)
	//B0 = index B1=Channel A, B2=channelB
	//setup port A as output (for motor controller)
	out8(cnt_ddr, 0b00000010);

	//Declare storage for our peripheral container
	robot_data bot;

	init_motor(&bot.motorA,cnt_port_a,1,2,0); //motor controller out on pins A1,A2 pwm out on pin A0
	motor_setMode(&bot.motorA,forward);

	init_motor(&bot.motorB,cnt_port_a,4,5,3);//motor controller out on pins A4,A5 pwm out on pin A3
	motor_setMode(&bot.motorB,forward);

	//Accelerometer on ADC 0,1,2 for x,y,z respectively sample at 400hz
	init_accelerometer(&bot.accelerometer,cnt_base,0,1,2,2500000);


	///Set up PID
	bot.setpoint = 84.3;
	init_pid(&bot.PID, &bot.setpoint, &bot.accelerometer.yztheta,0.13,0.0001,-0.004);
	start_pid(&bot.PID);

	//Start motor output thread
	pthread_t output_thread;
	pthread_create(&output_thread,NULL,&Output, (void*)&bot);

	///Start user input thread.
	pthread_t input_thread;
	pthread_create(&input_thread,NULL,&Input, (void*)&bot);




	//create the log file based on the PID constants
	char fname[30];
	sprintf(fname,"log-%0.4lf-%0.4lf-%0.4lf.csv",bot.PID.pk,bot.PID.ik,bot.PID.dk);


	FILE* fp= fopen(fname,"w+");

	for(;;){

		//updateEncoder(&encoderA);
		fprintf(fp,"%lf,%lf\r\n",pipe_get_average(&bot.accelerometer.yztheta),pipe_get_average(&bot.PID.output)*30);
		usleep(100000);

	}
	return EXIT_SUCCESS;
}


