/*
 * iodefs.h
 *
 *  Created on: Feb 25, 2012
 *      Author: aah9038
 */

#ifndef IODEFS_H_
#define IODEFS_H_

#include <stdint.h>
#include <semaphore.h>
#include <hw/inout.h>
#include <stdio.h>

typedef struct{
	double value;
	sem_t mutex;
}pipeline_dat;

void setPin(uintptr_t port, char pin, char value);

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

#endif /* IODEFS.H */