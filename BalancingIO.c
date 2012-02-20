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


typedef struct{
	pthread_t pid_thread;
	double* input;		//pointer to input value
	double* setpoint;	//pointer to setpoint value
	sem_t	input_sem;	//input semaphore. Unblocked when input changes
	double e[3]; 		//error history
	double u[3]; 		//output history
	double pk,ik,dk; 	//PID constants
	sem_t 	output_sem;	//output semaphore. Unblocked when output changes
	double output;

}pid_data;

typedef struct{

	short values[16];

	uintptr_t base;
	double x;
	double y;
	double z;

	uintptr_t adc_port;
	char x_pin;
	char y_pin;
	char z_pin;

	pthread_t input_thread; //thread running that fills this structure

}accel_dat;


typedef struct{
	uintptr_t cnt_port;	//port that encoder is hooked to
	char last_read;		//copy of last port read for edge detection
	char read;		//current port read
	char debounce;		//used to debounce channel A
	char a_mask;	//bitmask for channel A pin
	char b_mask;	//bitmask for channel B pin
	char index_mask;
	int first_index;	//index that first index rising edge was found at.
	int position;		//current position encoder is at

}encoder_dat;


typedef struct{
	int high_time;
	int period;

	char currentOutput;
	uintptr_t cnt_port;	//output port
	char output_mask; //mask to determine what pins to write to
	char stop;

	pthread_t output_thread;

}pwm_args;

typedef enum {
	freewheel	= 0b00,
			forward 	= 0b01,
			backward	= 0b10,
			brake		= 0b11
}motor_mode;

/**
 * L298N motor controller driver
 *
 *
 */
typedef struct{
	uintptr_t cnt_port; //motor control ouput port
	char input_1_pin;
	char input_2_pin;
	motor_mode current_mode;
	pwm_args pwm;
}motor_t;



void pwm_setDuty(pwm_args* args, float duty);
double getAngle(accel_dat* data);
void* accelCallback(void * param);
void init_accelerometer(accel_dat* data, uintptr_t base, char x_pin, char y_pin, char z_pin, int nseconds);
void setPin(uintptr_t port, char pin, char value);
void motor_setSpeed(motor_t* motor, float speed);
void motor_setMode(motor_t* motor, motor_mode mode);
void init_motor(motor_t* motor, uintptr_t port, char input_1_pin, char input_2_pin, char pwmPin);
int initialize_handles();
void* pwm_thread(void* param);
void pwm_setDuty(pwm_args* args, float duty);
void startPWM(pwm_args* args,uintptr_t outputPort, char output_mask, int high_time, int period);
void initEncoder(encoder_dat* data, char index_pin, char chan_a_pin, char chan_b_pin, uintptr_t port );
void updateEncoder(encoder_dat* data);

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

void* accelCallback(void * param){
	accel_dat* data= (accel_dat*) param;


	sem_wait(&port_mutex);
	while ( in8(data->base+ATD_STATUS) & STS_BM){ //wait for the STS bit to clear
	}

	int values= in8(data->base+ATD_FIFO_DEPTH);//read in the number of values in the fifo

	int i;
	for (i=0;i<values && i < 16;i++){
		data->values[i]=in8(data->base+ATD_LSB) + (in8(data->base+ATD_MSB)<<8);
	}

	//reset the fifo
	out8(data->base+CMD_REGISTER, RSTFIFO);

	//start conversion for next time
	out8(data->base+CMD_REGISTER,ADC_TRIGGER_READ);
	sem_post(&port_mutex);

	printf("X:[%d]\r\n",data->values[(int)data->x_pin]-5496);
	printf("Z:[%d]\r\n",data->values[(int)data->z_pin]-5496);

	printf("theta: %lf\r\n",getAngle(data));

	return 0;
}


void init_accelerometer(accel_dat* data, uintptr_t base, char x_pin, char y_pin, char z_pin, int nseconds){

	char temp;

	data->base=base;

	data->x_pin=x_pin;
	data->y_pin=y_pin;
	data->z_pin=z_pin;

	sem_wait(&port_mutex);
	//enable scan and set gain to 0
	out8(base+ADC_GAIN_OFFSET, (ADC_SCANEN)|(ADC_GAIN_0) );

	//set scan range from 0 to 15
	out8(base+ADC_RANGE_OFFSET, (ADC_HIGH_15)|(ADC_LOW_0));

	//setup interrupt control register
	temp=in8(base+INTERRUPT_CONTROL_REGISTER);
	out8(base+INTERRUPT_CONTROL_REGISTER, temp & ~(ADC_CLK_EXTERNAL | ADC_INTERRUPT_ENABLE));
	//start conversion so it will be ready
	out8(base+CMD_REGISTER,ADC_TRIGGER_READ);

	sem_post(&port_mutex);

	struct sigevent event;
	timer_t timer;
	struct itimerspec value;

	SIGEV_THREAD_INIT(&event, &accelCallback, (void*) data, NULL);
	timer_create(CLOCK_REALTIME,&event, &timer);

	value.it_value.tv_sec=0;
	value.it_value.tv_nsec=nseconds;

	value.it_interval.tv_sec=0;
	value.it_interval.tv_nsec=nseconds;

	timer_settime(timer, 0, &value, NULL);


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


//Accepts a signed float from -1 to 1 that determines speed of the motor
void motor_setSpeed(motor_t* motor, float speed){

	if(speed <0.0){
		motor_setMode(motor, backward);
	}
	else{
		motor_setMode(motor, forward);
	}
	pwm_setDuty(&motor->pwm, fabs(speed));
}

void motor_setMode(motor_t* motor, motor_mode mode){
	motor->current_mode=mode;

	setPin(motor->cnt_port,motor->input_1_pin,(mode & 0b01));
	setPin(motor->cnt_port,motor->input_2_pin,(mode & 0b10));

	/*char current=in8(motor->cnt_port);
	  current = current & ~((1<<motor->input_1_pin) | (1<<motor->input_2_pin));

	  out8(motor->cnt_port, current| (((mode & 0b01)<<motor->input_1_pin) |((mode & 0b10)<<(motor->input_2_pin-1))) );*/
}


void init_motor(motor_t* motor, uintptr_t port, char input_1_pin, char input_2_pin, char pwmPin){

	motor->cnt_port=port;
	motor->input_1_pin=input_1_pin;
	motor->input_2_pin=input_2_pin;

	motor_setMode(motor,freewheel);

	startPWM(&motor->pwm,port,0x1<<pwmPin,0,255);//pwm out on pin A0

}

void init_pid(pid_data* pid, double* input, double* output, double pk, double ik, double dk) {
	pid->e = {0, 0, 0};
	pid->u = {0, 0, 0};
	pid->input = input;
	pid->output = output;
	pid->pk = pk;
	pid-> ik = ik;
	pid-> dk = dk;
	sem_init(*input_sem,0,0);
	sem_init(*output_sem,0,0);
}

void start_pid(pid_data* pid) {
	pthread_create(&pid->pid_thread, void, pid_thread);
}

void update_input(pid_data* pid, int channel) {
	//set reading from specified channel
	out8(cnt_input_chan, channel);
	//write to STRTAD (base + 0) OR 1000 0000 (to start A/D conversion).
	out8(cnt_base, 0x80);
	//wait until base + 3 < 128
	while (in8(cnt_input_rng) & 0x80); //wait for conversion to finish before proceeding
	//then read A/D at base + 0(lsb) and Base + 1 (msb)
	int LSB = in8(cnt_base);
	int8_t MSB = in8(cnt_base_plus);

	//convert and add together lsb + msb*256
	double read_val = MSB*256 + LSB;
	//convert into analog number (using scale)
	read_val = read_val /32768 * 10;
	//save to input
	pid->input = read_val;
}
/*typedef struct{

	pthread_t pid_thread;
	double* input;
	double* setpoint;
	//TODO: input mutex
	double e[3]; //error history
	double u[3]; //output history
	double pk,ik,dk; //PID constants

	//TODO: output mutex
	double output;

}pid_data;*/
void* pid_thread(void* param){	
	pid_data* pid = (pid_data*)param;

	while(1){ //run forever. TODO: figure out exit case?
		sem_wait(pid->input_sem); //wait for input to change

		pid->u[2] = pid->u[1];	//propogate hitory forward
		pid->u[1] = pid->u[0];
		pid->u[0] = pid->output;

		pid->e[2] = pid->e[1];
		pid->e[1] = pid->e[0];
		pid->e[0] = pid->setpoint - pid->input;

		//update output as per pid equation
		//u(k+1) = u(k) + e(k+1)(pk+ik+dk) - e(k)(pk+2dk) + e(k-1)dk
		pid->output = pid->u[2] + pid->input*(pk+ik+dk) - pid->e[0]*(pk+2*dk) + e[1]*dk;

		sem_post(pid->output_sem);//notify system that output has changed
	}

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

void* pwm_thread(void* param){
	pwm_args* args = (pwm_args*)param;

	int count=0;

	sem_wait(&port_mutex);
	args->currentOutput= in8(args->cnt_port);
	out8(args->cnt_port,args->currentOutput | (args->output_mask)); //set high
	sem_post(&port_mutex);

	struct itimerspec value;


	while(!args->stop){
		if(args->high_time>0){
			sem_wait(&port_mutex);
			args->currentOutput= in8(args->cnt_port);
			out8(args->cnt_port,args->currentOutput | (args->output_mask)); //set high
			sem_post(&port_mutex);
			value.it_value.tv_nsec = args->high_time*10000;
			value.it_value.tv_sec = 0;
			nanosleep(&value,NULL); //more friendly
			//nanospin(&value);//much faster.. up it_value
		}
		sem_wait(&port_mutex);
		args->currentOutput= in8(args->cnt_port);
		out8(args->cnt_port,args->currentOutput & ~(args->output_mask)); //set low
		sem_post(&port_mutex);

		value.it_value.tv_nsec = (args->period-args->high_time)*10000;
		value.it_value.tv_sec = 0;
		nanosleep(&value,NULL); //more friendly
		//nanospin(&value);//much faster.. up it_value

	}

	//set the output to 0 before leaving
	out8(args->cnt_port,args->currentOutput & ~(args->output_mask)); //set all the controlled pins to low

	return 0;
}

void pwm_setDuty(pwm_args* args, float duty){

	args->high_time=(int) (duty * args->period);

	//clip high time to 0 and period
	args->high_time = args->high_time < 0? 0 :args->high_time;
	args->high_time = args->high_time > args->period ? args->period : args->high_time;

}


void startPWM(pwm_args* args,uintptr_t outputPort, char output_mask, int high_time, int period){

	args->high_time=high_time;
	args->period=period;
	args->cnt_port=outputPort;
	args->output_mask=output_mask;
	args->currentOutput=0;
	args->stop=0;

	pthread_create(&args->output_thread,NULL, &pwm_thread,(void*)args); //create a new thread for pwm


}


void initEncoder(encoder_dat* data, char index_pin, char chan_a_pin, char chan_b_pin, uintptr_t port ){
	data->a_mask=0x01<<chan_a_pin;
	data->b_mask=0x01<<chan_b_pin;
	data->index_mask=0x01<<index_pin;

	data->cnt_port=port;
	data->debounce=0;
	data->position=0;
	data->last_read=0;
	data->read=0;
	data->first_index=0;

}

void updateEncoder(encoder_dat* data){
	data->last_read=data->read;
	data->read = in8(data->cnt_port); //read encoder port

	if( ((data->a_mask)& data->read) &&! ((data->a_mask)& data->last_read)){ //rising edge for channel A

		if (((data->b_mask) & data->read) && data->debounce){ //if B is high
			data->position++;
		}
		else if (data->debounce){ //b is low
			data->position--;
		}
		data->debounce = 0;
	}

	if(((data->b_mask)& data->read) &&! ((data->b_mask)& data->last_read)){ //rising edge for channel B
		data->debounce = 1;
	}

	if(((data->index_mask)& data->read) &&! ((data->index_mask)& data->last_read)){ //rising edge for index
		if (data->first_index){

		}else{ //set first index
			data->first_index=data->position;
		}
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

	//accel_dat accelerometer;
	//init_accelerometer(&accelerometer,cnt_base,0,1,2,5000000); //Accelerometer on ADC 0,1,2 for x,y,z respectively

	for(;;){

		/*last_input=input;
		  input = in8(cnt_port_b); //read in port B

		  if( ((chan_a_mask)& input) &&! ((chan_a_mask)& last_input)){ //rising edge for channel A

		  if (((chan_b_mask) & input) && debounce){ //if B is high
		  position++;
		  }
		  else if (debounce){ //b is low
		  position--;
		  }
		  debounce = 0;
		  }

		  if( ((chan_b_mask)& input) &&! ((chan_b_mask)& last_input)){ //rising edge for channel B
		  debounce = 1;

		  }*/
		char a=getchar();

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
			printf("%d\n",encoderA.position);
		}


	}
	return EXIT_SUCCESS;
}



void * Input (void* arg){
	/*	while(1){

	//sem_wait
	sem_wait(&inputSemaphore);

	//Reading input from purplebox Analog input

	//write to STRTAD (base + 0) OR 1000 0000 (to start A/D conversion).
	out8(cnt_base, 0x80);

	//wait until base + 3 < 128
	while (in8(cnt_input_rng) & 0x80); //wait for conversion to finish before proceeding
	//then read A/D at base + 0(lsb) and Base + 1 (msb)
	int LSB = in8(cnt_base);
	int8_t MSB = in8(cnt_base_plus);

	//convert and add together
	//	->  lsb + msb*256
	double read = MSB*256 + LSB;
	//convert into analog number (using scale)
	read = read /32768 * 10;
	sem_post(&controlSemaphore);
	}//while true*/
}
