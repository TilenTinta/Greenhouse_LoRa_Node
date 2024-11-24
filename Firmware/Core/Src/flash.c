/*
 * File: flash.c
 *
 *  Created on: Aug. 31, 2024
*   Author: Tinta T.
 */

#include "flash.h"
#include "string.h"
#include "stdio.h"

uint8_t bytes_temp[4];

void Convert_To_Str(uint32_t *Data, char *Buf);

float Bytes2float(uint8_t * ftoa_bytes_temp);


/* STM32F103C8Tx have 128 PAGES (Page 0 to Page 127) of 1 KB each = 128 KB Flash Memory */

static uint32_t GetPage(uint32_t Address)
{
	// FLASH_PAGE_SIZE is STM function
	for (int indx=0; indx<FLASH_PAGE_NO; indx++)
	{
		if((Address < (0x08000000 + (FLASH_PAGE_SIZE *(indx+1))) ) && (Address >= (0x08000000 + FLASH_PAGE_SIZE*indx)))
	    {
			return (0x08000000 + FLASH_PAGE_SIZE*indx);
	    }
	}

  return 0;
}


// Conversion of float variable to 4 bytes
void float2Bytes(uint8_t * ftoa_bytes_temp, float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }
}


// Conversion of 4 byte variable to float
float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}


// Write data to MCU flash
uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar = 0;

	HAL_FLASH_Unlock();

	uint32_t StartPage = GetPage(StartPageAddress);
	uint32_t EndPageAdress = StartPageAddress + numberofwords*4;
	uint32_t EndPage = GetPage(EndPageAdress);

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = StartPage;
	EraseInitStruct.NbPages     = ((EndPage - StartPage) / FLASH_PAGE_SIZE) + 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	{
		return HAL_FLASH_GetError ();
	}

	while (sofar < numberofwords)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress, Data[sofar]) == HAL_OK)
		{
			StartPageAddress += 4;
			sofar++;
		}
		else
		{
			 return HAL_FLASH_GetError ();
		}
	}

	HAL_FLASH_Lock();

	return 0;
}


// Read data from MCU flash
void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{
		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}


// Convert 4 byte data to string
void Convert_To_Str(uint32_t *Data, char *Buf)
{
	int numberofbytes = ((strlen((char *)Data) / 4) + ((strlen((char *)Data) % 4) != 0)) * 4;

	for (int i = 0; i < numberofbytes; i++)
	{
		Buf[i] = Data[i/4]>>(8 * (i % 4));
	}
}


// Write data to MCU flash - float number -> one 4 byte number
void Flash_Write_NUM(uint32_t StartSectorAddress, float Num)
{
	float2Bytes(bytes_temp, Num);

	Flash_Write_Data(StartSectorAddress, (uint32_t *)bytes_temp, 1);
}


// Read data from MCU flash - one 4 byte number -> float number
float Flash_Read_NUM (uint32_t StartSectorAddress)
{
	uint8_t buffer[4];
	float value;

	Flash_Read_Data(StartSectorAddress, (uint32_t *)buffer, 1);
	value = Bytes2float(buffer);
	return value;
}
