//#include <cstdlib>
//#include <iostream.h>

#include <time.h>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <stdint.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/netmgr.h>
#include <sys/neutrino.h>
#include <semaphore.h>
#include <math.h>

#include "iodefs.h"
#include "pid.h"
#include "motor.h"
#include "accelerometer.h"
#include "encoder.h"
#include "pipeline.h"
#include "BalancingRobot.h"

void * Control(void* arg);
void * Output(void* arg);
void * Input(void* arg);
void * UserInput (void* arg);
void RunControlAlgorithm();

int initialize_handles();


/* ******************************************************************
 * Initiaize register handles
 *****************************************************************/

int initialize_handles(){

	sem_init(&port_mutex,0,1);

	cnt_base = mmap_device_io( PORT_LENGTH, CNT_BASE );
	if ( cnt_base == MAP_DEVICE_FAILED ) {
		perror( "cnt_base map failed" ); return EXIT_FAILURE;
	}

	cnt_base_plus = mmap_device_io(PORT_LENGTH, CNT_BASE +1);
	if ( cnt_base_plus == MAP_DEVICE_FAILED ) {
		perror( "cnt_base_plus map failed" ); return EXIT_FAILURE;
	}

	cnt_input_chan = mmap_device_io(PORT_LENGTH, INPUT_CHAN);
	if ( cnt_input_chan == MAP_DEVICE_FAILED ) {
		perror( "cnt_input_chan map failed" ); return EXIT_FAILURE;
	}

	cnt_output_chan_high = mmap_device_io(PORT_LENGTH, OUTPUT_CHAN_HIGH);
	if ( cnt_output_chan_high == MAP_DEVICE_FAILED ) {
		perror( "cnt_input_chan map failed" ); return EXIT_FAILURE;
	}

	cnt_output_chan_low = mmap_device_io(PORT_LENGTH, OUTPUT_CHAN_LOW);
	if ( cnt_output_chan_low == MAP_DEVICE_FAILED ) {
		perror( "cnt_input_chan map failed" ); return EXIT_FAILURE;
	}

	cnt_input_rng = mmap_device_io(PORT_LENGTH, INPUT_RNG);
	if ( cnt_input_rng == MAP_DEVICE_FAILED ) {
		perror( "cnt_input_rng map failed" ); return EXIT_FAILURE;
	}

	cnt_ddr = mmap_device_io(PORT_LENGTH, DDR);
	if ( cnt_ddr == MAP_DEVICE_FAILED ) {
		perror( "cnt_ddr map failed" ); return EXIT_FAILURE;
	}

	cnt_port_a = mmap_device_io(PORT_LENGTH, PORT_A);
	if ( cnt_port_a == MAP_DEVICE_FAILED ) {
		perror( "cnt_port_a map failed" ); return EXIT_FAILURE;
	}

	cnt_port_b = mmap_device_io(PORT_LENGTH, PORT_B);
	if ( cnt_port_b == MAP_DEVICE_FAILED ) {
		perror( "cnt_port_b map failed" ); return EXIT_FAILURE;
	}

	sem_wait(& port_mutex);
	//Set reading from channel 0
	//out8(cnt_input_chan, 0x00);

	//Set writing to channel ?
	//int reg = in8(cnt_output_chan_high);
	//bitwise and 00111111 with what's currently in register
	//(to set bit 6 and 7 to select channel 0).
	//out8(cnt_output_chan_high, (reg & 0x3f));
	//wait for it to settle????

	//Set input range to +/- 10V
	out8(cnt_input_rng, 0x00);
	//wait for it to settle????
	sem_post(& port_mutex);
	//SEE PAGE 68 of Athena manual SECTION 13.4



	return EXIT_SUCCESS;
}

void * Input(void* arg){
	pid_dat* pid = (pid_dat*)arg;
	double* ptr;
	double val;

	while (1) {

			char variable = getchar();
			char operator = getchar();
			scanf("%lf", &val);
			while (getchar() != '\n'){}

			switch(variable){
			case 'p':
				ptr = &(pid->pk);
				break;
			case 'i':
				ptr = &(pid->ik);
				break;
			case 'd':
				ptr = &(pid->dk);
				break;
			case 's':
				ptr = pid->setpoint;
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
		printf( "Can't get root permissions\n");
		return -1;
	} else {
		//printf("Got root permissions\n");
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


	robot_data bot;

	init_motor(&bot.motorA,cnt_port_a,1,2,0); //motor controller out on pins A1,A2 pwm out on pin A0
	motor_setMode(&bot.motorA,forward);

	init_motor(&bot.motorB,cnt_port_a,4,5,3);//motor controller out on pins A4,A5 pwm out on pin A3
	motor_setMode(&bot.motorB,forward);

	//Accelerometer on ADC 0,1,2 for x,y,z respectively sample at 400hz
	init_accelerometer(&bot.accelerometer,cnt_base,0,1,2,2500000);


	bot.setpoint = 83.8;
	init_pid(&bot.PID, &bot.setpoint, &bot.accelerometer.yztheta,0.13,0.0001,-0.001);
	start_pid(&bot.PID);

	pthread_t input_thread;
	pthread_create(&input_thread,NULL,&Input, (void*)&bot.PID);

	pthread_t output_thread;
	pthread_create(&output_thread,NULL,&Output, (void*)&bot);

	char fname[30];

	sprintf(fname,"log-%0.4lf-%0.4lf-%0.4lf.csv",bot.PID.pk,bot.PID.ik,bot.PID.dk);


	FILE* fp= fopen(fname,"w+");

	for(;;){

		//updateEncoder(&encoderA);
		//printf("%lf,%lf\r\n",pipe_get_average(&bot.accelerometer.yztheta),pipe_get_average(&bot.PID.output)*30);
		fprintf(fp,"%lf,%lf\r\n",pipe_get_average(&bot.accelerometer.yztheta),pipe_get_average(&bot.PID.output)*30);
		usleep(100000);

	}
	return EXIT_SUCCESS;
}


