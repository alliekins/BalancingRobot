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

#include "pid.h"
#include "motor.h"
#include "accelerometer.h"
#include "encoder.h"

void * Control(void* arg);
//void * Output(void* arg);
void Output();
void * Input(void* arg);
void * UserInput (void* arg);
int initialize_handles();
void RunControlAlgorithm();

const size_t PORT_LENGTH = 1;

#define CNT_BASE  0x280
#define ADC_GAIN_OFFSET 3
#define RSTFIFO 	(0x8)

#define ATD_STATUS 	3
#define STS_BM		(0x80)

#define ATD_FIFO_DEPTH	6

#define ATD_MSB		1
#define ATD_LSB		0

#define CMD_REGISTER		0
#define RST_DAC				0x20
#define RST_BRD				0x40
#define ADC_TRIGGER_READ	(0x80)

#define ADC_RANGE_OFFSET 	2
#define ADC_HIGH_15			(0xF<<4)
#define ADC_LOW_0			(0x0)

#define ADC_SCANEN 		(0b100)
#define ADC_GAIN_0 		(0b00)


#define INTERRUPT_CONTROL_REGISTER	4
#define ADC_CLK_EXTERNAL 		(0b10000)
#define ADC_INTERRUPT_ENABLE		(0b1)



const uint64_t INPUT_CHAN = (CNT_BASE+2);
const uint64_t OUTPUT_CHAN_LOW = (CNT_BASE+6);
const uint64_t OUTPUT_CHAN_HIGH = (CNT_BASE+7);
const uint64_t DDR = (CNT_BASE+11);
const uint64_t PORT_A = (CNT_BASE+8);
const uint64_t PORT_B = (CNT_BASE+9);


const uint64_t INPUT_RNG = (CNT_BASE+3);
const uint64_t CNT1_CLK_100KHZ = 0x40;
//register handles

//TODO: need to protect these ports with mutexes.
sem_t port_mutex;
uintptr_t cnt_base;
uintptr_t cnt_base_plus;
uintptr_t cnt_input_rng;
uintptr_t cnt_input_chan;
uintptr_t cnt_output_chan_high;
uintptr_t cnt_output_chan_low;
uintptr_t cnt_ddr;
uintptr_t cnt_port_a;
uintptr_t cnt_port_b;

double getAngle(accel_dat* data);

void setPin(uintptr_t port, char pin, char value);
int initialize_handles();

double getAngle(accel_dat* data){

#define PI 3.14159265

	double x = (double)data->values[(int)data->x_pin]-5496;
	double y = (double)data->values[(int)data->y_pin]-5496;
	double z = (double)data->values[(int)data->z_pin]-5496;


	double len = sqrt((x * x) + (z * z)+ (y*y) );

	y=y/len;
	x=x/len;
	z=z/len;

	//to fix this error in eclipse, make sure to include "-lm" in the linker options



	return (PI/2-atan(y/z))*180/PI;

}

void setPin(uintptr_t port, char pin, char value){
	sem_wait(& port_mutex);
	char current=in8(port);
	if(value){
		out8(port, current | (1<<pin));
		printf("set %x\n", pin);
	}else{
		out8(port,current & ~(1<<pin));
		printf("clear %x\n",pin);
	}
	sem_post(&port_mutex);


}



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
	init_accelerometer(&accelerometer,cnt_base,0,1,2,5000000); //Accelerometer on ADC 0,1,2 for x,y,z respectively

	pid_data PID;
	double setpoint = 90.0;
	init_pid(&PID, &setpoint, &accelerometer.yztheta,.05,0.0,0.0);
	start_pid(&PID);




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

		updateEncoder(&encoderA);

		if(++count > 1000){
			count =0;
			printf("set: %lf\n",PID.output.value);
			motor_setSpeed(&motorA,PID.output.value);
			motor_setSpeed(&motorB,PID.output.value);
		}


	}
	return EXIT_SUCCESS;
}


