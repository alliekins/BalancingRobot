/*
 * pipeline.c
 *
 *  Created on: Feb 26, 2012
 *      Author: aml6195
 */

#include <semaphore.h>
#include <stdlib.h>
#include "pipeline.h"



void init_pipeline(pipeline_dat* dat, int size){

	sem_init(&dat->mutex,0,0);
	sem_init(&dat->read_mutex,0,1);


	dat->values= malloc(size*sizeof(double));

	if (dat->values == NULL){
		perror("Memory allocation error");
	}

	dat->size=size;
	dat->index=0;
	int i;

	for (i=0;i<dat->size;i++){
		dat->values[i]=0.0;
	}
}

void add_to_pipeline(pipeline_dat* dat, double val){

	sem_wait(&dat->read_mutex);

	if (dat->index >  dat->size || dat->size > 100){
		perror("array out of bounds");
	}

	dat->lastInput=val;
	dat->values[dat->index]= val;
	dat->index = (dat->index+1) % dat->size;

	sem_post(&dat->read_mutex);

	//if we have filled the pipeline, post to the mutex, saying we are done.
	if(dat->index==0){
		sem_post(&dat->mutex);
	}
}


double pipe_get_average(pipeline_dat* dat){

	sem_wait(&dat->read_mutex);
	int i;
	double average=0.0;
	for (i=0;i<(dat->size);i++){
		average+=dat->values[i];
	}
	average /= dat->size;

	sem_post(&dat->read_mutex);

	return average;
}

