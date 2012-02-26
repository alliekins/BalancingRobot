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

void * Control(void* arg);
//void * Output(void* arg);
void Output();
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
	pid_data* pid = (pid_data*)arg;
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

	unsigned char input=0;
	unsigned char last_input=0;
	unsigned char debounce=0;

	int position=0;
	int count=0;

	encoder_dat encoderA;
	initEncoder(&encoderA, 0, 1, 2, cnt_port_b);


	motor_t motorA;
	init_motor(&motorA,cnt_port_a,1,2,0); //motor controller out on pins A1,A2 pwm out on pin A0
	motor_setMode(&motorA,forward);

	motor_setSpeed(&motorA,.75);

	motor_t motorB;
	init_motor(&motorB,cnt_port_a,4,5,3);//motor controller out on pins A4,A5 pwm out on pin A3
	motor_setMode(&motorB,forward);
	motor_setSpeed(&motorB,.75);

	accel_dat accelerometer;
	//Accelerometer on ADC 0,1,2 for x,y,z respectively sample at 400hz
	init_accelerometer(&accelerometer,cnt_base,0,1,2,2500000);

	pid_data PID;
	double setpoint = 82.5;
	init_pid(&PID, &setpoint, &accelerometer.yztheta,.19,0.0,0.0);
	start_pid(&PID);

	pthread_t input_thread;
	pthread_create(&input_thread,NULL,&Input, (void*)&PID);

	for(;;){

		char a='p';

		if (a=='a'){
			motor_setMode(&motorA,backward);
		}
		if (a=='d'){
			motor_setMode(&motorA,forward);
		}
		if (a=='e'){
			motor_setMode(&motorA,freewheel);
		}
		if (a=='b'){
			motor_setMode(&motorA,brake);
		}

		//updateEncoder(&encoderA);

		sem_wait(&PID.output.mutex);

		if(++count > 1000){
			count =0;
			printf("theta: %lf set: %lf\r\n",accelerometer.yztheta.average,PID.output.value);
		}
		motor_setSpeed(&motorA,PID.output.value);
		motor_setSpeed(&motorB,PID.output.value);

	}
	return EXIT_SUCCESS;
}


