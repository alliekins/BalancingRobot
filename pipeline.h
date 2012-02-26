/*
 * pipeline.h
 *
 *  Created on: Feb 26, 2012
 *      Author: aml6195
 */

#ifndef PIPELINE_H_
#define PIPELINE_H_

typedef struct{
	double value;
	double values[100];
	int index;
	int size;
	double average;
	sem_t mutex;
}pipeline_dat;

void init_pipeline(pipeline_dat* dat, int size);

void add_to_pipeline(pipeline_dat* dat, double val);

#endif /* PIPELINE_H_ */
