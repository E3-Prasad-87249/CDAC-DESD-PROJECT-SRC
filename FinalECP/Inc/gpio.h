/*
 * gpio.h
 *
 *  Created on: Feb 5, 2025
 *      Author: sunbeam
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "mm_addresses.h"

// Peripherals regdef structure
typedef struct
{
	uint32_t GPIOx_MODER;		// Offset: 0x00
	uint32_t GPIOx_OTYPER;		// Offset: 0x04
	uint32_t GPIOx_OSPEEDR;		// Offset: 0x08
	uint32_t GPIOx_PUPDR;		// Offset: 0x0C
	uint32_t GPIOx_IDR;			// Offset: 0x10
	uint32_t GPIOx_ODR;			// Offset: 0x14
	uint32_t GPIOx_BSRR;		// Offset: 0x18
	uint32_t GPIOx_LCKR;		// Offset: 0x1C
	uint32_t GPIOx_AFRL;		// Offset: 0x20
	uint32_t GPIOx_AFRH;		// Offset: 0x24
}GPIO_RegDef_st;
//GPIO_RegDef_st *pGPIORegDefSt = (pGPIORegDefSt*)(GPIOA_BASE_ADDR);


typedef struct
{
	uint32_t moder0:2;
	uint32_t moder1:2;
	uint32_t moder2:2;
	uint32_t moder3:2;
	uint32_t moder4:2;
	uint32_t moder5:2;
	uint32_t moder6:2;
	uint32_t moder7:2;
	uint32_t moder8:2;
	uint32_t moder9:2;
	uint32_t moder10:2;
	uint32_t moder11:2;
	uint32_t moder12:2;
	uint32_t moder13:2;
	uint32_t moder14:2;
	uint32_t moder15:2;
}GPIOx_MODER_bfst;
GPIOx_MODER_bfst *pGPIOA_MODER = (GPIOx_MODER_bfst*)GPIOA_MODER_ADDR;
GPIOx_MODER_bfst *pGPIOD_MODER = (GPIOx_MODER_bfst*)GPIOD_MODER_ADDR;

typedef struct
{
	uint32_t idr0:1;
	uint32_t idr1:1;
	uint32_t idr2:1;
	uint32_t idr3:1;
	uint32_t idr4:1;
	uint32_t idr5:1;
	uint32_t idr6:1;
	uint32_t idr7:1;
	uint32_t idr8:1;
	uint32_t idr9:1;
	uint32_t idr10:1;
	uint32_t idr11:1;
	uint32_t idr12:1;
	uint32_t idr13:1;
	uint32_t idr14:1;
	uint32_t idr15:1;
	uint32_t reserved:16;

}GPIOx_IDR_bfst;
GPIOx_IDR_bfst *pGPIOA_IDR = (GPIOx_IDR_bfst*)GPIOA_IDR_ADDR;

typedef struct
{
	uint32_t odr0:1;
	uint32_t odr1:1;
	uint32_t odr2:1;
	uint32_t odr3:1;
	uint32_t odr4:1;
	uint32_t odr5:1;
	uint32_t odr6:1;
	uint32_t odr7:1;
	uint32_t odr8:1;
	uint32_t odr9:1;
	uint32_t odr10:1;
	uint32_t odr11:1;
	uint32_t odr12:1;
	uint32_t odr13:1;
	uint32_t odr14:1;
	uint32_t odr15:1;
	uint32_t reserved:16;
}GPIOx_ODR_bfst;
GPIOx_ODR_bfst *pGPIOD_ODR = (GPIOx_ODR_bfst*)GPIOD_ODR_ADDR;

#endif /* GPIO_H_ */
