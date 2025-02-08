/*
 * main3.h
 *
 *  Created on: Feb 7, 2025
 *      Author: sunbeam
 */
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "main.h"

void someDelay()
{
	for(int i=0; i<500000; i++);
}

/*int main3(void)
{
	pRCC->RCC_AHB1ENR.AHB1ENR = 0x00100009;

	//pGPIOD->GPIOx_MODER.GPIOx_MODER = (uint32_t)0b0101010100000000000000000000000000;
	pGPIOD->GPIOx_MODER.GPIOx_MODER = 0x55000000;
	pGPIOA->GPIOx_MODER.GPIOx_MODER = 0xA8000000;

	for(;;)
	{
		if((pGPIOA->GPIOx_IDR.GPIOx_IDR & 0x00000001) == 1){
			someDelay();
			pGPIOD->GPIOx_ODR.GPIOx_ODR ^= 0x0000f000;
		}
	}
	return 0;
}*/

/*int main2(void)
{
	pRCC->RCC_AHB1ENR.AHB1ENRbf.gpioa_en = 1;
	pRCC->RCC_AHB1ENR.AHB1ENRbf.gpiod_en = 1;

	pGPIOD->GPIOx_MODER.MODERbf.moder12 = 1;
	pGPIOD->GPIOx_MODER.MODERbf.moder13 = 1;
	pGPIOD->GPIOx_MODER.MODERbf.moder14 = 1;
	pGPIOD->GPIOx_MODER.MODERbf.moder15 = 1;
	pGPIOA->GPIOx_MODER.MODERbf.moder0 = 0;

	for(;;)
	{
		if(pGPIOA->GPIOx_IDR.IDRbf.idr0 == 1){
			someDelay();
			pGPIOD->GPIOx_ODR.ODRbf.odr12 = ~pGPIOD->GPIOx_ODR.ODRbf.odr12;
			pGPIOD->GPIOx_ODR.ODRbf.odr13 = ~pGPIOD->GPIOx_ODR.ODRbf.odr13;
			pGPIOD->GPIOx_ODR.ODRbf.odr14 = ~pGPIOD->GPIOx_ODR.ODRbf.odr14;
			pGPIOD->GPIOx_ODR.ODRbf.odr15 = ~pGPIOD->GPIOx_ODR.ODRbf.odr15;
		}
	}
	return 0;
}*/

int main(void)
{
	pRCC->RCC_AHB1ENR |= (1 << 3);
	pRCC->RCC_AHB1ENR |= (1 << 0);

	pGPIOD->GPIOx_MODER &= ~( 3 << 24);
	pGPIOD->GPIOx_MODER &= ~( 3 << 26);
	pGPIOD->GPIOx_MODER &= ~( 3 << 28);
	pGPIOD->GPIOx_MODER &= ~( 3 << 30);
	pGPIOD->GPIOx_MODER |=  ( 1 << 24);
	pGPIOD->GPIOx_MODER |=  ( 1 << 26);
	pGPIOD->GPIOx_MODER |=  ( 1 << 28);
	pGPIOD->GPIOx_MODER |=  ( 1 << 30);
	pGPIOA->GPIOx_MODER &= ~(3 << 0);

	for(;;)
	{
		//read the pin status of the pin PA0 (GPIOA INPUT DATA REGISTER)
		uint8_t  pinStatus = (uint8_t)(pGPIOA->GPIOx_IDR & 0x1); //zero out all other bits except bit 0
		if(pinStatus){
			someDelay();
				pGPIOD->GPIOx_ODR ^= (1<<12); // toggle 12th bit(PORTD.12)
				pGPIOD->GPIOx_ODR ^= (1<<13); // toggle 13th bit(PORTD.12)
				pGPIOD->GPIOx_ODR ^= (1<<14); // toggle 14th bit(PORTD.12)
				pGPIOD->GPIOx_ODR ^= (1<<15); // toggle 15th bit(PORTD.12)
		}
	}
	return 0;
}
