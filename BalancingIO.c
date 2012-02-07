//#include <cstdlib>
//#include <iostream.h>
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

#define MY_PULSE_CODE   _PULSE_CODE_MINAVAIL

typedef union {
	struct _pulse   pulse;

} my_message_t;

void * Control(void* arg);
//void * Output(void* arg);
void Output();
void * Input(void* arg);
void * UserInput (void* arg);
int initialize_handles();
void RunControlAlgorithm();

const size_t PORT_LENGTH = 1;
#define CNT_BASE  0x280
const uint64_t INPUT_CHAN = (CNT_BASE+2);
const uint64_t OUTPUT_CHAN_LOW = (CNT_BASE+6);
const uint64_t OUTPUT_CHAN_HIGH = (CNT_BASE+7);
const uint64_t DDR = (CNT_BASE+11);
const uint64_t PORT_A = (CNT_BASE+8);
const uint64_t PORT_B = (CNT_BASE+9);

const uint64_t INPUT_RNG = (CNT_BASE+3);
const uint64_t CNT1_CLK_100KHZ = 0x40;
//register handles
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
	uintptr_t cnt_port;	//port that encoder is hooked to
	char lastRead;		//copy of last port read for edge detection
	char read;		//current port read
	char debounce;		//used to debounce channel A
	char channel_a_mask;	//bitmask for channel A pin
	char channel_b_mask;	//bitmask for channel B pin
	int first_index;	//index that first index rising edge was found at.
	int position;		//current position encoder is at	

}encoder_dat;




/* ******************************************************************
 * Initiaize register handles
 *****************************************************************/
int initialize_handles(){

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


	//Set reading from channel 0
	out8(cnt_input_chan, 0x00);

	//Set writing to channel ?
	int reg = in8(cnt_output_chan_high);
	//bitwise and 00111111 with what's currently in register
	//(to set bit 6 and 7 to select channel 0).
	out8(cnt_output_chan_high, (reg & 0x3f));
	//wait for it to settle????

	//Set input range to +/- 10V
	out8(cnt_input_rng, 0x00);
	//wait for it to settle????

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


	out8(cnt_base, 0x20); //reset ATD

	//setup port B as input (for encoder)
	//A0 = index A1=Channel A, A2=channelB
#define chan_a_mask 0x02
#define chan_b_mask 0x04
	//setup port A as output (for motor controller)
	out8(cnt_ddr, 0b00000010);

	unsigned char input=0;
	unsigned char last_input=0;
	unsigned char debounce=0;

	int position=0;
	int count=0;

	for(;;){
		last_input=input;
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

		}


		if(++count > 1000){
			count =0;
			printf("%d\n",position);
		}


	}
	return EXIT_SUCCESS;
}

void updateEncoder(encoder_dat* data){
	data->last_read=data->read;
	data->read = in8(data->cnt_port); //read encoder port

	if( ((channel_a_mask)& data->read) &&! ((channel_a_mask)& last_data->read)){ //rising edge for channel A

		if (((channel_b_mask) & data->read) && data->debounce){ //if B is high
			data->position++;
		}
		else if (data->debounce){ //b is low
			data->position--;
		}
		data->debounce = 0;
	}

	if(((channel_b_mask)& data->read) &&! ((channel_b_mask)& last_data->read)){ //rising edge for channelnel B
		data->debounce = 1;

	}
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
