/*
 * lab1math.h
 *
 *  Created on: Jan 22, 2026
 *      Author: wjaso
 */

#ifndef SRC_LAB1MATH_H_
#define SRC_LAB1MATH_H_

// find whatever using ASM
uint8_t asmTrans(float x0, float omega, float phi, float epsilon, uint16_t maxIter, float *root);

// find transcendwhatever using C
uint8_t transcend(float x0, float omega, float phi, float epsilon, uint16_t maxIter, float *root);

// finding sqrt using newty-raphsy
uint8_t newtonRaphsonSqrt(float x, float epsilon, uint16_t maxIter, float *sqrtX);

// finding sqrt using asm
extern void asmSqrt(float x, float *sqrtX);

// finding max
extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);
void cMax(float *array, uint32_t size , float *max, uint32_t *maxIndex);


#endif /* SRC_LAB1MATH_H_ */
