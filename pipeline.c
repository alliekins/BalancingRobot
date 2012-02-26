/*
 * pipeline.c
 *
 *  Created on: Feb 26, 2012
 *      Author: aml6195
 */

#include <semaphore.h>
#include "pipeline.h"


void init_pipeline(pipeline_dat* dat, int size){
	sem_init(&dat->mutex,0,0);

	if (size > 100){
		perror("Error: pipeline size max 100");

	}

	dat->size=size;
	dat->index=0;
	int i;
	for (i=0;i<dat->size;i++){
		dat->values[i]=0.0;
	}
	dat->average=0.0;
}

void add_to_pipeline(pipeline_dat* dat, double val){

	if (dat->index >  dat->size || dat->size > 100){
		perror("array out of bounds");
	}

	dat->value=val;

	dat->values[dat->index]= val;
	dat->index = (dat->index+1) % dat->size;

	if(dat->index==0){
		sem_post(&dat->mutex);

	}


	int i;
	dat->average=0;
	for (i=0;i<(dat->size);i++){
		dat->average+=val;
	}
	dat->average /= dat->size;

}
