/*
 * 001_Printf.c
 *
 *  Created on: Jan 23, 2020
 *      Author: nemanja
 */

#include "stm32f446xx.h"

#if !defined(__SOFT_FPU__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize FPU before use."
#endif


int main(void)
{
    return 0;
}
