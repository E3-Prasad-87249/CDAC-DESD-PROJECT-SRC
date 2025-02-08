/*
 * main.h
 *
 *  Created on: Feb 7, 2025
 *      Author: sunbeam
 */

#ifndef MAIN_H_
#define MAIN_H_

//#include "mm_addresses.h"
#define RCC_BASE_ADDR 		(0x40023800UL)
#define GPIOA_BASE_ADDR 	(0x40020000UL)
#define GPIOD_BASE_ADDR 	(0x40020C00UL)
/*
 * Bit Feild Structures
 */
typedef struct
{
	volatile uint32_t gpioa_en:1;
	volatile uint32_t gpiob_en:1;
	volatile uint32_t gpioc_en:1;
	volatile uint32_t gpiod_en:1;
	volatile uint32_t gpioe_en:1;
	volatile uint32_t gpiof_en:1;
	volatile uint32_t gpiog_en:1;
	volatile uint32_t gpioh_en:1;
	volatile uint32_t gpioi_en:1;
	volatile uint32_t reserved1:3;
	volatile uint32_t crc_en:1;
	volatile uint32_t reserved2:5;
	volatile uint32_t bkpsram_en:1;
	volatile uint32_t reserved3:1;
	volatile uint32_t ccmdataram_en:1;
	volatile uint32_t dma1_en:1;
	volatile uint32_t dma2_en:1;
	volatile uint32_t reserved4:2;
	volatile uint32_t ethmac_en:1;
	volatile uint32_t ethmactx_en:1;
	volatile uint32_t ethmacrx_en:1;
	volatile uint32_t ethmacptp_en:1;
	volatile uint32_t otghs_en:1;
	volatile uint32_t otghsulpi_en:1;
	volatile uint32_t reserved5:1;
}RCC_AHB1ENR_bfst;
// union for full register or bitfeild access
typedef union
{
	volatile uint32_t AHB1ENR;
	RCC_AHB1ENR_bfst AHB1ENRbf;
}RCC_AHB1ENR_un;


/*
 * Peripherals regdef structure(s)
 */
typedef struct
{
	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_PLLCFGR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR;
	volatile uint32_t RCC_AHB1RSTR;
	volatile uint32_t RCC_AHB2RSTR;
	volatile uint32_t RCC_AHB3RSTR;
	uint32_t reserved1;
	volatile uint32_t RCC_APB1RSTR;
	volatile uint32_t RCC_APB2RSTR;
	uint32_t reserved2;
	uint32_t reserved3;
	volatile uint32_t RCC_AHB1ENR;		//
	//RCC_AHB1ENR_un RCC_AHB1ENR;
	volatile uint32_t RCC_AHB2ENR;
	volatile uint32_t RCC_AHB3ENR;
	uint32_t reserved4;
	volatile uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_APB2ENR;
	uint32_t reserved5;
	uint32_t reserved6;
	volatile uint32_t RCC_AHB1LPENR;
	volatile uint32_t RCC_AHB2LPENR;
	volatile uint32_t RCC_AHB3LPENR;
	uint32_t reserved7;
	volatile uint32_t RCC_APB1LPENR;
	volatile uint32_t RCC_APB2LPENR;
	uint32_t reserved8;
	uint32_t reserved9;
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	uint32_t reserved10;
	uint32_t reserved11;
	volatile uint32_t RCC_SSCGR;
	volatile uint32_t RCC_PLLI2SCFGR;
}RCC_RegDef_st;

typedef struct
{
	volatile uint32_t moder0:2;
	volatile uint32_t moder1:2;
	volatile uint32_t moder2:2;
	volatile uint32_t moder3:2;
	volatile uint32_t moder4:2;
	volatile uint32_t moder5:2;
	volatile uint32_t moder6:2;
	volatile uint32_t moder7:2;
	volatile uint32_t moder8:2;
	volatile uint32_t moder9:2;
	volatile uint32_t moder10:2;
	volatile uint32_t moder11:2;
	volatile uint32_t moder12:2;
	volatile uint32_t moder13:2;
	volatile uint32_t moder14:2;
	volatile uint32_t moder15:2;
}GPIOx_MODER_bfst;
// union for full register or bitfeild access
typedef union
{
	volatile uint32_t GPIOx_MODER;
	GPIOx_MODER_bfst MODERbf;
}GPIOx_MODER_un;

typedef struct
{
	volatile uint32_t idr0:1;
	volatile uint32_t idr1:1;
	volatile uint32_t idr2:1;
	volatile uint32_t idr3:1;
	volatile uint32_t idr4:1;
	volatile uint32_t idr5:1;
	volatile uint32_t idr6:1;
	volatile uint32_t idr7:1;
	volatile uint32_t idr8:1;
	volatile uint32_t idr9:1;
	volatile uint32_t idr10:1;
	volatile uint32_t idr11:1;
	volatile uint32_t idr12:1;
	volatile uint32_t idr13:1;
	volatile uint32_t idr14:1;
	volatile uint32_t idr15:1;
	volatile uint32_t reserved:16;
}GPIOx_IDR_bfst;
// union for full register or bitfeild access
typedef union
{
	volatile uint32_t GPIOx_IDR;
	GPIOx_IDR_bfst IDRbf;
}GPIOx_IDR_un;


typedef struct
{
	volatile uint32_t odr0:1;
	volatile uint32_t odr1:1;
	volatile uint32_t odr2:1;
	volatile uint32_t odr3:1;
	volatile uint32_t odr4:1;
	volatile uint32_t odr5:1;
	volatile uint32_t odr6:1;
	volatile uint32_t odr7:1;
	volatile uint32_t odr8:1;
	volatile uint32_t odr9:1;
	volatile uint32_t odr10:1;
	volatile uint32_t odr11:1;
	volatile uint32_t odr12:1;
	volatile uint32_t odr13:1;
	volatile uint32_t odr14:1;
	volatile uint32_t odr15:1;
	volatile uint32_t reserved:16;
}GPIOx_ODR_bfst;
// union for full register or bitfeild access
typedef union
{
	volatile uint32_t GPIOx_ODR;
	GPIOx_ODR_bfst ODRbf;
}GPIOx_ODR_un;


typedef struct
{
	volatile uint32_t GPIOx_MODER;		// Offset: 0x00				//
	//GPIOx_MODER_un GPIOx_MODER;
	volatile uint32_t GPIOx_OTYPER;		// Offset: 0x04
	volatile uint32_t GPIOx_OSPEEDR;		// Offset: 0x08
	volatile uint32_t GPIOx_PUPDR;		// Offset: 0x0C
	volatile uint32_t GPIOx_IDR;			// Offset: 0x10			//
	//GPIOx_IDR_un GPIOx_IDR;
	volatile uint32_t GPIOx_ODR;			// Offset: 0x14			//
	//GPIOx_ODR_un GPIOx_ODR;
	volatile uint32_t GPIOx_BSRR;		// Offset: 0x18
	volatile uint32_t GPIOx_LCKR;		// Offset: 0x1C
	volatile uint32_t GPIOx_AFRL;		// Offset: 0x20
	volatile uint32_t GPIOx_AFRH;		// Offset: 0x24
}GPIOx_RegDef_st;


#define pRCC ((RCC_RegDef_st*)RCC_BASE_ADDR)
#define pGPIOA ((GPIOx_RegDef_st*)GPIOA_BASE_ADDR)
#define pGPIOD ((GPIOx_RegDef_st*)GPIOD_BASE_ADDR)

#endif /* MAIN_H_ */
