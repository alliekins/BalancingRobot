/*
 * accelerometer.h
 *
 *  Created on: Feb 24, 2012
 *      Author: aah9038
 */

#ifndef ACCEL_H_
#define ACCEL_H_

#include "iodefs.h"
#include "pipeline.h"
#include <hw/inout.h>
#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>
#include <math.h>

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

	pipeline_dat xztheta;
	pipeline_dat yztheta;
	pipeline_dat xytheta;

	pthread_t input_thread; //thread running that fills this structure

}accel_dat;

void* accelCallback(void * param);
void init_accelerometer(accel_dat* data, uintptr_t base, char x_pin, char y_pin, char z_pin, int nseconds);
double getAngle(accel_dat* data);

#endif /* ACCEL_H_ */
