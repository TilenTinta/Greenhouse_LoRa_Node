/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <string.h>
#include "BME280.h"
#include "flash.h"
#include "rfm95.h"
#include "keys.h" // These file needs to be created - standard header file with three #defines (DEVICE_ID, DEVICE_ADDRESS, APPSKEY, NWKSKEY)

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOOST_EN_Pin GPIO_PIN_0
#define BOOST_EN_GPIO_Port GPIOA
#define AN_BAT_Pin GPIO_PIN_1
#define AN_BAT_GPIO_Port GPIOA
#define AN_E_HUM_Pin GPIO_PIN_2
#define AN_E_HUM_GPIO_Port GPIOA
#define EHUM_PWR_Pin GPIO_PIN_3
#define EHUM_PWR_GPIO_Port GPIOA
#define SPI_CS_EX_Pin GPIO_PIN_4
#define SPI_CS_EX_GPIO_Port GPIOA
#define D_VBAT_EN_Pin GPIO_PIN_0
#define D_VBAT_EN_GPIO_Port GPIOB
#define DIO5_Pin GPIO_PIN_1
#define DIO5_GPIO_Port GPIOB
#define DIO5_EXTI_IRQn EXTI1_IRQn
#define UNUSED3_Pin GPIO_PIN_2
#define UNUSED3_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_8
#define RESET_GPIO_Port GPIOA
#define DIO2_Pin GPIO_PIN_11
#define DIO2_GPIO_Port GPIOA
#define SIM_RST_Pin GPIO_PIN_12
#define SIM_RST_GPIO_Port GPIOA
#define DIO1_Pin GPIO_PIN_15
#define DIO1_GPIO_Port GPIOA
#define DIO0_Pin GPIO_PIN_3
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI3_IRQn
#define DIO4_Pin GPIO_PIN_4
#define DIO4_GPIO_Port GPIOB
#define DIO3_Pin GPIO_PIN_5
#define DIO3_GPIO_Port GPIOB
#define SIM_ISR_Pin GPIO_PIN_8
#define SIM_ISR_GPIO_Port GPIOB
#define SIM_SLP_Pin GPIO_PIN_9
#define SIM_SLP_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LDO_OUT_U			3.15	// Measured voltage of the LDO used to calculate earth humidity
#define BAT_R1 				500000	// Resistor 1 in vultage divider for battery voltage
#define BAT_R2 				500000	// Resistor 2 in vultage divider for battery voltage

#define EARTH_HUM_DRY_VAL	595		// Return value of ADC for dry sensor (set for each sensor)
#define EARTH_HUM_WET_VAL	240		// Return value of ADC for wet sensor (set for each sensor)

#define SLEEP_MODE_STOP				// Comment that line if you want to put the device in standby mode (STOP mode keeps data in RAM)

#define STATE_INIT			0		// State of initialization - device power on
#define STATE_FIRST_CONN	1		// State where node gets first connection		///////// STANDBY MODE NOT TESTED!!! /////////
#define STATE_RUN			2		// State for main code
#define STATE_SEND			3		// State for sending data over LoRa or SIM
#define STATE_GO_SLEEP		4		// State where mcu is put to sleep
#define STATE_ERROR			5		// Init routine failed


// Required data for LoRa that must be saved - retentive
typedef struct{

	uint8_t 	data[10];

} LORA_Save;

// Data packet to be send over LoRa
typedef struct{

	uint8_t 	error;
	uint8_t 	errSendCnt;
	uint8_t 	battery;
	int16_t		air_temperature;
	uint8_t 	air_humidity;
	uint32_t 	air_pressure;
	uint8_t 	earth_humudity;

} LORA_Data;


// Saved analog measured values
typedef struct{

	uint8_t 	ADC_read;			// Trigger ADC reading
	uint8_t 	ADC_read_end;		// Flag when ADC read is finished
	uint8_t 	ADC_read_cnt; 		// Counter of how many ADC reads was taken

	uint32_t 	ADC_values[2];		// Value of ADC read with DMA

	float 		earth_hum[5];			// Measured humidity in ground
	float 		earth_humidity;		// Calculated average humidity in ground

	float 		bat_voltage[5];		// Battery voltage
	float 		battery_voltage;		// Calculated average battery voltage

} MEASUREMENTS;


float ADC_Read_Battery(uint32_t* ADC_value);
uint8_t ADC_Read_EHum(uint32_t* ADC_value, float* delta);
uint8_t ADC_Read_EHum(uint32_t* ADC_value, float* delta);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
