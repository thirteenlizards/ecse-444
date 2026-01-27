/*
 * cmax.c
 *
 *  Created on: not Jan 22, 2026
 *      Author: not wjaso
 *
 *  Newton-Raphson Method Source:
 *  R. Anstee, The Newton‑Raphson Method, Dept. of Mathematics, Univ. of British Columbia. Vancouver, BC, Canada. [Online]. Available: https://personal.math.ubc.ca/~anstee/math104/newtonmethod.pdf
 */
#include "main.h"
#include "lab1math.h"
#include <math.h>

uint8_t transcend(float x0, float omega, float phi, float epsilon, uint16_t maxIter, float *root) {

	float xCurrent = x0;
	float xNext, fVal, dfVal, diff;

	for (uint8_t i = 0; i < maxIter; i++) {

		// f(x) = x^2 - cos(omega*x + phi)
		fVal = (xCurrent*xCurrent) - cosf(omega * xCurrent + phi);

		// f'(x) = 2*x + omega*sin(omega*x + phi)
		dfVal = 2.0f*xCurrent + omega*sinf(omega*xCurrent + phi);

		// don't divide by 0 like LAST TIME
		if (dfVal == 0.0f) {
			return 1; // return error
		}


		// compute next x value
		xNext = xCurrent -fVal/dfVal;

		// check convergence btwn x values
		diff = xNext - xCurrent;
		if (fabs(diff) < epsilon) {
			*root = xNext; // yay worked
			return 0;
		}

		// prepare for next iteration
		xCurrent = xNext;
	}

	return 1; // didn't converge within maxIter
}
