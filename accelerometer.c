/*
 * accelerometer.c
 *
 *  Created on: Feb 24, 2012
 *      Author: aah9038
 */

#include "accelerometer.h"
#include "iodefs.h"

void* accelCallback(void * param){
	accel_dat* data= (accel_dat*) param;

	sem_wait(&port_mutex);
	while ( in8(data->base+ATD_STATUS) & STS_BM){ //wait for the STS bit to clear
	}

	int values= in8(data->base+ATD_FIFO_DEPTH);//read in the number of values in the fifo

	int i;
	for (i=0;i<values && i < 16;i++){
		data->values[i]=in8(data->base+ATD_LSB) + (in8(data->base+ATD_MSB)<<8);
	}

	//reset the fifo
	out8(data->base+CMD_REGISTER, RSTFIFO);

	//start conversion for next time
	out8(data->base+CMD_REGISTER,ADC_TRIGGER_READ);
	sem_post(&port_mutex);

	/*printf("X:[%d]\r\n",data->values[(int)data->x_pin]-5496);
	printf("Z:[%d]\r\n",data->values[(int)data->z_pin]-5496);

	printf("theta: %lf\r\n",getAngle(data));*/


	getAngle(data);
	//printf("theta %lf\r\n",data->yztheta.value );

	return 0;
}


void init_accelerometer(accel_dat* data, uintptr_t base, char x_pin, char y_pin, char z_pin, int nseconds){

	char temp;

	data->base=base;

	data->x_pin=x_pin;
	data->y_pin=y_pin;
	data->z_pin=z_pin;

	init_pipeline(&(data->xztheta),20);
	init_pipeline(&(data->yztheta),20);
	init_pipeline(&(data->xytheta),20);

	sem_wait(&port_mutex);
	//enable scan and set gain to 0
	out8(base+ADC_GAIN_OFFSET, (ADC_SCANEN)|(ADC_GAIN_0) );

	//set scan range from 0 to 15
	//out8(base+ADC_RANGE_OFFSET, (ADC_HIGH_15)|(ADC_LOW_0));

	//set scan range from 0 to 2
	out8(base+ADC_RANGE_OFFSET, (ADC_HIGH_2)|(ADC_LOW_0));

	//setup interrupt control register
	temp=in8(base+INTERRUPT_CONTROL_REGISTER);
	out8(base+INTERRUPT_CONTROL_REGISTER, temp & ~(ADC_CLK_EXTERNAL | ADC_INTERRUPT_ENABLE));
	//start conversion so it will be ready
	out8(base+CMD_REGISTER,ADC_TRIGGER_READ);

	sem_post(&port_mutex);

	struct sigevent event;
	timer_t timer;
	struct itimerspec value;

	SIGEV_THREAD_INIT(&event, &accelCallback, (void*) data, NULL);
	timer_create(CLOCK_REALTIME,&event, &timer);

	value.it_value.tv_sec=0;
	value.it_value.tv_nsec=nseconds;

	value.it_interval.tv_sec=0;
	value.it_interval.tv_nsec=nseconds;

	timer_settime(timer, 0, &value, NULL);

}

double getAngle(accel_dat* data){

#define PI 3.14159265

	double x = (double)data->values[(int)data->x_pin]-581;
	double y = (double)data->values[(int)data->y_pin]-5496-x; //remove x (0) to remove noise
	double z = (double)data->values[(int)data->z_pin]-5496-x; //


	double len = sqrt((x * x) + (z * z)+ (y*y) );

	y=y/len;
	x=x/len;
	z=z/len;

	//to fix this error in eclipse, make sure to include "-lm" in the linker options

	add_to_pipeline(&data->xztheta,((PI/2-atan(x/z))*180/PI));
	add_to_pipeline(&data->yztheta,((PI/2-atan(y/z))*180/PI));
	add_to_pipeline(&data->xytheta,((PI/2-atan(x/y))*180/PI));

	return (PI/2-atan(y/z))*180/PI;

}
