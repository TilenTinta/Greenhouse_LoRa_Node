/*
 * eeprom.c
 *
 *  Created on: Sep 15, 2025
 *  Author: Tilen Tinta
 */

#include "eeprom.h"


// Converts a float to a 4-byte array.
static void float2Bytes(uint8_t *byte_array, float float_variable) {
    union {
        float a;
        uint8_t bytes[4];
    } thing;
    thing.a = float_variable;
    memcpy(byte_array, thing.bytes, 4);
}


// Converts a 4-byte array to a float.
static float bytes2float(uint8_t *byte_array) {
    union {
        float a;
        uint8_t bytes[4];
    } thing;
    memcpy(thing.bytes, byte_array, 4);
    return thing.a;
}


// Write data to eeprom at specified address
HAL_StatusTypeDef EEPROM_Write_Data(uint32_t StartAddress, uint32_t *Data, uint16_t numberofwords)
{
	// Clear any sticky error flags from a previous cycle (survive reset)
	__HAL_FLASH_CLEAR_FLAG(
	    FLASH_FLAG_EOP    |
	    FLASH_FLAG_WRPERR |
	    FLASH_FLAG_PGAERR |
	    FLASH_FLAG_SIZERR |
	#if defined(FLASH_SR_RDERR)
	    FLASH_FLAG_RDERR  |
	#endif
	#if defined(FLASH_SR_OPTVERRUSR)
	    FLASH_FLAG_OPTVERRUSR |
	#endif
	    FLASH_FLAG_OPTVERR
	);


    // Check if the address range is valid for the EEPROM
    uint32_t endAddress = StartAddress + (numberofwords * 4) - 1;
    if (StartAddress < FLASH_EEPROM_BASE || endAddress > FLASH_EEPROM_END) {
        return HAL_ERROR; // Address is out of the valid EEPROM range
    }

    HAL_FLASHEx_DATAEEPROM_Unlock();

    for (uint16_t i = 0; i < numberofwords; i++)
    {
        uint32_t currentAddress = StartAddress + (i * 4);
        uint32_t currentData = Data[i];

        // Program one word at a time
        if (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, currentAddress, currentData) != HAL_OK)
        {
            HAL_FLASHEx_DATAEEPROM_Lock();
            return HAL_ERROR;
        }
    }

    HAL_FLASHEx_DATAEEPROM_Lock();
    return HAL_OK;
}


void EEPROM_Read_Data(uint32_t StartAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
    for (uint16_t i = 0; i < numberofwords; i++)
    {
        // Direct memory read from EEPROM
        RxBuf[i] = *(__IO uint32_t *)(StartAddress + (i * 4));
    }
}


void EEPROM_Write_Float(uint32_t StartAddress, float Num)
{
    uint8_t bytes_temp[4];
    float2Bytes(bytes_temp, Num);
    EEPROM_Write_Data(StartAddress, (uint32_t *)bytes_temp, 1);
}


float EEPROM_Read_Float(uint32_t StartAddress)
{
    uint8_t buffer[4];
    EEPROM_Read_Data(StartAddress, (uint32_t *)buffer, 1);
    return bytes2float(buffer);
}
