/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include <string.h>
#include "BME280.h"
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
#define SPI1_CS_EX_Pin GPIO_PIN_4
#define SPI1_CS_EX_GPIO_Port GPIOA
#define D_VBAT_EN_Pin GPIO_PIN_0
#define D_VBAT_EN_GPIO_Port GPIOB
#define DIO5_Pin GPIO_PIN_1
#define DIO5_GPIO_Port GPIOB
#define DIO5_EXTI_IRQn EXTI1_IRQn
#define UNUSED1_Pin GPIO_PIN_2
#define UNUSED1_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define RESET_Pin GPIO_PIN_8
#define RESET_GPIO_Port GPIOA
#define DIO2_Pin GPIO_PIN_11
#define DIO2_GPIO_Port GPIOA
#define DIO1_Pin GPIO_PIN_15
#define DIO1_GPIO_Port GPIOA
#define DIO0_Pin GPIO_PIN_3
#define DIO0_GPIO_Port GPIOB
#define DIO0_EXTI_IRQn EXTI3_IRQn
#define DIO4_Pin GPIO_PIN_4
#define DIO4_GPIO_Port GPIOB
#define DIO3_Pin GPIO_PIN_5
#define DIO3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LDO_USE								// Comment this line if you don't use LDO on PCB - power from battery
#define BAT_U				4.2				// Set voltage of the battery
#define LDO_OUT_U			3.6				// Measured voltage of the LDO used to calculate earth humidity
#define BAT_R1 				100				// Resistor 1 in voltage divider for battery voltage
#define BAT_R2 				100				// Resistor 2 in voltage divider for battery voltage

#define EARTH_HUM_DRY_VAL	595				// Return value of ADC for dry sensor (set for each sensor)
#define EARTH_HUM_WET_VAL	240				// Return value of ADC for wet sensor (set for each sensor)

//#define SLEEP_MODE_STOP						// Comment that line if you want to put the device in standby mode (STOP mode keeps data in RAM)
#define USE_EEPROM							// Use EEPROM instead of flash to save data

// Uncomment only one option
//#define SLEEP_PERIOD_TEST					// Amount of time for MCU to sleep - 10sec
//#define SLEEP_PERIOD_ONE_MINUTE			// Amount of time for MCU to sleep - 1min
//#define SLEEP_PERIOD_15_MINUTES			// Amount of time for MCU to sleep - 15min
#define SLEEP_PERIOD_HALF_HOUR				// Amount of time for MCU to sleep - 30min
//#define SLEEP_PERIOD_ONE_HOUR				// Amount of time for MCU to sleep - 1h
//#define SLEEP_PERIOD_CUSTOM		45		// Amount of time for MCU to sleep - custom value

#define RTC_CLOCK_REF		32768			// Frequency of the RTC clock

#define READ_VBAT_PERIOD	24				// Read the battery only every N HOURS to decrease the power consumption
#define VBAT_CRYTHICAL		2.4				// Minimum battery voltage allowed - must be set

#define STATE_INIT			0				// State of initialization - device power on
#define STATE_FIRST_CONN	1				// State where node gets first connection		///////// STANDBY MODE NOT TESTED!!! /////////
#define STATE_RUN			2				// State for main code
#define STATE_SEND			3				// State for sending data over LoRa or SIM
#define STATE_GO_SLEEP		4				// State where mcu is put to sleep
#define STATE_ERROR			5				// Init routine failed


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

	uint8_t		init_end;					// Flag for init complete

	uint8_t 	ADC_read;					// Trigger ADC reading
	uint8_t 	ADC_read_end;				// Flag when ADC read is finished
	uint8_t 	ADC_read_cnt; 				// Counter of how many ADC reads was taken

	uint32_t 	ADC_values[2];				// Value of ADC read with DMA

	float 		earth_hum[5];				// Measured humidity in ground
	float 		earth_humidity;				// Calculated average humidity in ground

	uint8_t		bat_period_counter;				// Counter for control battery reading (decrease power consumption)
	float 		bat_voltage[5];				// Battery voltage
	float 		battery_voltage;			// Calculated average battery voltage

} MEASUREMENTS;


float ADC_Read_Battery(uint32_t* ADC_value);
uint8_t ADC_Read_EHum(uint32_t* ADC_value, float* delta);
uint8_t ADC_Read_EHum(uint32_t* ADC_value, float* delta);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
