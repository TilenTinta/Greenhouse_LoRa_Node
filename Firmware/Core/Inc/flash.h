/*
 * flash.h
 *
 *  Created on: Aug. 31, 2024
 *  Author: Tinta T.
 */

#include "main.h"

#define FLASH_START_ADDR 	0x0807F800		// Size: 1K
#define FLASH_PAGE_NO		128				// Number of pages

#ifndef FLASH_H_
#define FLASH_H_


uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);

void Flash_Write_NUM(uint32_t StartSectorAddress, float Num);

float Flash_Read_NUM(uint32_t StartSectorAddress);

#endif