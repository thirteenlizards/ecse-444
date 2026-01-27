/*
 * cmax.c
 *
 *  Created on: Jan 22, 2026
 *      Author: not wjaso
 */

/*
 * Newton-Raphson Equation:
 * S. G. Johnson, Square Roots via Newton’s Method, MIT Course 18.335 lecture handout, Massachusetts Institute of Technology, Feb. 4, 2015. [Online]. Available: https://math.mit.edu/~stevenj/18.335/newton-sqrt.pdf
 */

#include "main.h"
#include "lab1math.h"
#include <math.h>
#include <stdlib.h>

uint8_t newtonRaphsonSqrt(float x, float epsilon, uint16_t maxIter, float *sqrtX) {

	float xSqrt_current = x/2.0f;
	float xSqrt_next = 0;
	float diff;


	for (uint8_t i = 1; i < maxIter; i++) {

		//Sqrt_next = (((xSqrt_current) + x) / xSqrt_current)/2;

		xSqrt_next = 0.5*(xSqrt_current + (x/xSqrt_current));

		diff = xSqrt_next - xSqrt_current;

		if (abs(diff) < epsilon) {
			*sqrtX = xSqrt_next; 	// store result
			return 0; 				// returned correctly
		}

		xSqrt_current = xSqrt_next; // set up for next iteration
	}

	return 1; // did not converge


	// some garbage
}
