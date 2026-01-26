/*
* asmmax.s
*/

// unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmSqrt, which is expected by lab1math.h
.global asmSqrt

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata


/**
* void asmSqrt(float x, float *sqrtX);
*
* S0 = input value, float(x)
* R0 = pointer to output value, *sqrtX
*/

asmSqrt:

	VSQRT.F32 S0, S0	// Perform squre root function on input value x in S0, and put result sqrtX back in S0
	VSTR.F32 S0, [R0] 	// Store sqrtX in S0 to memory address given in R1
	BX LR 				// Return
