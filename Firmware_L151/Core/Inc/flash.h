/*
 * flash.h
 *
 *  Created on: Aug. 31, 2024
 *  Author: Tinta T.
 */

#include "main.h"

// MCU with 64kB of flash
#define FLASH_START_ADDR 	0x0800FC00     	// Size: 256B (1024bytes; 0-511 pages; page0 = 0x08000000, page1 = 0x08000400; 0x0800F800 = 496page) - Page 496 (496 * 128 = 63488 = 0xF800)
#define FLASH_PAGE_NO		256             // Number of pages (64KB / 128B)
#define FLASH_PAGE_SIZE_L1  256         	// Page size is 256 bytes for STM32L1xx

#ifndef FLASH_H_
#define FLASH_H_


uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);

void Flash_Write_NUM(uint32_t StartSectorAddress, float Num);

float Flash_Read_NUM(uint32_t StartSectorAddress);

#endif
