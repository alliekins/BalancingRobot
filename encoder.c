/*
 * encoder.c
 *
 *  Created on: Feb 24, 2012
 *      Author: aah9038
 */

#include "encoder.h"


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
