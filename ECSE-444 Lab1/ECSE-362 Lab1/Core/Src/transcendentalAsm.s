/*
* asmmax.s
*/

// unified indicates that we're using a mix of different ARM instructions,
// e.g., 16-bit Thumb and 32-bit ARM instructions may be present (and are)
.syntax unified

// .global exports the label asmMax, which is expected by lab1math.h
.global asmTrans
.extern arm_cos_f32
.extern arm_sin_f32

// .section marks a new section in assembly. .text identifies it as source code;
// .rodata marks it as read-only, setting it to go in FLASH, not SRAM
.section .text.rodata

/**
* uint8_t transcend(float x0, float omega, float phi, float epsilon, uint16_t maxIter, float *root) {
*
* S0 = x0, initial guess for x
* S1 = omega
* S2 = phi
* S3 = epsilon
* R0 = maxIter
* R1 = pointer to root
*/

asmTrans:

// R2 = loop counter
// S4 = fVal
// S5 = dfVal
// S9 = xCurrent (starting value is x0)
// S6 = xNext
// S7 = diff
// S8 = imm. register
// S10 = omega*x + phi
// S11 = imm. register
// S12 = 0 register
// S13 = 2 register
// S14 = 1 register

	VMOV S9, S0 // move xCurrent = x0 from S0 to S9
	MOV R2, #0 // set loop counter to 0
	VLDR.F32 S12, =zero
	VLDR.F32 S13, =two

loop:

// Increment loop counter
	ADD R2, R2, #1

// f(x) = x^2 - cos(omega*x + phi)

	// calculate omega*x
	VMUL.F32 S8, S1, S9 // VMUL Sd Sn Sm

	// calculate (omega*x) + phi
	VADD.F32 S10, S8, S2 // VADD Sd Sn Sm

	// calculate cos((omega*x) + phi)
	VMOV S0, S10 	// move function argument into S0
	BL arm_cos_f32	// call cosine function, output put in S0
	VMOV S8, S0		// move output from S0 into S9

	// calculate x^2
	VMUL.F32 S11, S9, S9

	// calculate fVal = x^2 - cos(omega*x + phi)
	VSUB.F32 S4, S11, S8


// f'(x) = 2*x + omega*sin(omega*x + phi)

	// calculate sin(omega*x + phi)
	VMOV S0, S10		// move funciton argument into S0
	BL arm_sin_f32  // call sine function, output put into S0
	VMOV S8, S0		// move output from S0 into S8

	// calculate omega*sin(omega*x + phi)
	VMUL.F32 S8, S1, S8

	// calculate 2*x
	VMUL.F32 S11, S9, S13

	// calculate dfVal = 2*x + omega*sin(omega*x + phi)
	VADD.F32 S5, S11, S8

// Compute next x value

	// compute fVal/dfVal
	VDIV.F32 S8, S4, S5

	// compute xNext = xCurrent -fVal/dfVal
	VSUB.F32 S6, S9, S8

// Check epsilon

	// diff = xNext - xCurrent;
	VSUB.F32 S8, S6, S9

	// fabs(diff)
	VABS.F32 S8, S8

	// if (fabs(diff) < epsilon)
	VCMP.F32 S8, S3
	BLT success // if comparison succeeds, done

// Check number of iterations
	CMP R2, R0
	BGE fail

// Else, prepare for next iteration
	// xCurrent = xNext;
	VMOV S6, S9
	B loop



success:
	// *root = xNext;
	// store xNext at memory location for the root
	VSTR.F32 S6, [R1]
	B end


fail:
	VSTR.F32 S12, [R1]
	B end


end:
	BX LR

two:
	.float 2.0

zero:
	.float 0.0




