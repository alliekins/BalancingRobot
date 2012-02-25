/*
 * encoder.h
 *
 *  Created on: Feb 24, 2012
 *      Author: aah9038
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

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

void initEncoder(encoder_dat* data, char index_pin, char chan_a_pin, char chan_b_pin, uintptr_t port );
void updateEncoder(encoder_dat* data);

#endif /* ENCODER_H_ */
