/*
 * eeprom.h
 *
 *  Created on: Sep 15, 2025
 *  Author: Tilen Tinta
 *
 *  Description: A standalone module to handle read/write operations
 *               on the STM32L1xx series internal Data EEPROM.
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "main.h" // Includes stm32l1xx_hal.h

/**
 * @brief Writes an array of 32-bit words to the Data EEPROM.
 * @param StartAddress The starting address in the EEPROM. Must be 4-byte aligned.
 *                     (e.g., FLASH_EEPROM_BASE, FLASH_EEPROM_BASE + 4, etc.)
 * @param Data Pointer to the array of words to write.
 * @param numberofwords The number of words to write from the array.
 * @retval HAL_StatusTypeDef HAL_OK if successful, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef EEPROM_Write_Data(uint32_t StartAddress, uint32_t *Data, uint16_t numberofwords);

/**
 * @brief Reads an array of 32-bit words from the Data EEPROM.
 * @param StartAddress The starting address in the EEPROM to read from.
 * @param RxBuf Pointer to a buffer where the read data will be stored.
 * @param numberofwords The number of words to read.
 */
void EEPROM_Read_Data(uint32_t StartAddress, uint32_t *RxBuf, uint16_t numberofwords);

/**
 * @brief Writes a single float value to the Data EEPROM.
 * @param StartAddress The EEPROM address to write to.
 * @param Num The float value to write.
 */
void EEPROM_Write_Float(uint32_t StartAddress, float Num);

/**
 * @brief Reads a single float value from the Data EEPROM.
 * @param StartAddress The EEPROM address to read from.
 * @retval The float value read from memory.
 */
float EEPROM_Read_Float(uint32_t StartAddress);


#endif /* INC_EEPROM_H_ */
