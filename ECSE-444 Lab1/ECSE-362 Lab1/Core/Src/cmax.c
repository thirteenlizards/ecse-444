/*
 * cmax.c
 *
 *  Created on: Jan 22, 2026
 *      Author: wjaso
 */
#include "main.h"
#include "lab1math.h"

void cMax(float *array, uint32_t size , float *max, uint32_t *maxIndex){
	(*max) = array[0];
	(*maxIndex) = 0;
	for(int i=0; i < (size); i++){
		if (array[i] > *max){
			(*max) = array[i];
			(*maxIndex) = i;
		}
	}
}


