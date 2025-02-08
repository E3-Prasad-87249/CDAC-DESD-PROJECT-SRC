/*
 * rcc.h
 *
 *  Created on: Feb 5, 2025
 *      Author: sunbeam
 */

#ifndef RCC_H_
#define RCC_H_

#include "mm_addresses.h"

typedef struct
{
	uint32_t gpioa_en:1;
	uint32_t gpiob_en:1;
	uint32_t gpioc_en:1;
	uint32_t gpiod_en:1;
	uint32_t gpioe_en:1;
	uint32_t gpiof_en:1;
	uint32_t gpiog_en:1;
	uint32_t gpioh_en:1;
	uint32_t gpioi_en:1;
	uint32_t reserved1:3;
	uint32_t crc_en:1;
	uint32_t reserved2:5;
	uint32_t bkpsram_en:1;
	uint32_t reserved3:1;
	uint32_t ccmdataram_en:1;
	uint32_t dma1_en:1;
	uint32_t dma2_en:1;
	uint32_t reserved4:2;
	uint32_t ethmac_en:1;
	uint32_t ethmactx_en:1;
	uint32_t ethmacrx_en:1;
	uint32_t ethmacptp_en:1;
	uint32_t otghs_en:1;
	uint32_t otghsulpi_en:1;
	uint32_t reserved5:1;
}RCC_AHB1ENR_bfst;
RCC_AHB1ENR_bfst *pRCC_AHB1ENR = (RCC_AHB1ENR_bfst*)RCC_AHB1ENR_ADDR;


#endif /* RCC_H_ */
