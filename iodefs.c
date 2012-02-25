/*
 * iodefs.c
 *
 *  Created on: Feb 25, 2012
 *      Author: aah9038
 */

#include "iodefs.h"

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
