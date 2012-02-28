/*
 * pipeline.h
 *
 *  Created on: Feb 26, 2012
 *      Author: aml6195
 */

#ifndef PIPELINE_H_
#define PIPELINE_H_

typedef struct{
	double lastInput;
	double* values;
	int index;
	int size;
	//double average; use accessor
	sem_t read_mutex;

	sem_t mutex;
}pipeline_dat;

void init_pipeline(pipeline_dat* dat, int size);

void add_to_pipeline(pipeline_dat* dat, double val);

double pipe_get_average(pipeline_dat* dat);

#endif /* PIPELINE_H_ */
