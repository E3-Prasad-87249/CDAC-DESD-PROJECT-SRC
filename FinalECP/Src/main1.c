/*
 * main1.c
 *
 *  Created on: Feb 5, 2025
 *      Author: sunbeam
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void someDelay()
{
	for(int i=0; i<500000; i++);
}

int main(void)
{
	uint32_t  volatile *const pClkCtrlReg =   (uint32_t*)0x40023830; // *pRCC_AHB1ENR
	uint32_t  volatile *const pPortDModeReg = (uint32_t*)0x40020C00;
	uint32_t  volatile *const pPortDOutReg =  (uint32_t*)0x40020C14;
	uint32_t volatile *const pPortAModeReg = (uint32_t*)0x40020000;
	uint32_t const volatile *const pPortAInReg =   (uint32_t*)0x40020010;

	// 1. enable the clock for GPOID , GPIOA peripherals in the AHB1ENR
	*pClkCtrlReg |= ( 1 << 3);
	*pClkCtrlReg |= ( 1 << 0);

	// 2.
	// configuring PD12,13,14 & 15 as output
	*pPortDModeReg &= ~( 3 << 24);
	*pPortDModeReg &= ~( 3 << 26);
	*pPortDModeReg &= ~( 3 << 28);
	*pPortDModeReg &= ~( 3 << 30);
	// make 24th, 26th, 28th, 30th bit position as 1 (SET)
	*pPortDModeReg |= ( 1 << 24);
	*pPortDModeReg |= ( 1 << 26);
	*pPortDModeReg |= ( 1 << 28);
	*pPortDModeReg |= ( 1 << 30);
	//Configure PA0 as input mode (GPIOA MODE REGISTER)
	*pPortAModeReg &= ~(3 << 0);

	// 3. business logic
	for(;;)
	{
		//read the pin status of the pin PA0 (GPIOA INPUT DATA REGISTER)
		uint8_t  pinStatus = (uint8_t)(*pPortAInReg & 0x1); //zero out all other bits except bit 0
		/*if(pinStatus){
			//turn on the LED
			*pPortDOutReg |= ( 1 << 12);
		}else{
			//turn off the LED
			*pPortDOutReg &= ~( 1 << 12);
		}*/
		if(pinStatus){
			someDelay();
			*pPortDOutReg ^= (1<<12); // toggle 12th bit(PORTD.12)
			*pPortDOutReg ^= (1<<13); // toggle 13th bit(PORTD.12)
			*pPortDOutReg ^= (1<<14); // toggle 14th bit(PORTD.12)
			*pPortDOutReg ^= (1<<15); // toggle 15th bit(PORTD.12)
		}
	}
}


