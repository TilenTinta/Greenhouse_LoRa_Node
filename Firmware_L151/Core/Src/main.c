/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

BME280 				bme280;
MEASUREMENTS 		measurements;
LORA_Data 			lora_data;
rfm95_handle_t 		rfm95_handle;

RTC_TimeTypeDef 	time;
RTC_DateTypeDef 	date;
RTC_AlarmTypeDef 	sAlarm;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* VARIABLES */
uint8_t awake = 1;				// Detect wake up event
uint8_t state = STATE_INIT;		// state value for state machine
volatile uint32_t tim3_tick_msb = 0;

/* MY FUNCTIONS */
void RFM95W_Struct_Init(rfm95_handle_t* rfm95_handle);
static uint8_t random_int(uint8_t max);
static uint32_t get_precision_tick();
void wake_up_from_standby_handler(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    // Check if the MCU woke up from Standby mode //
  	#ifndef SLEEP_MODE_STOP
  	  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB)) // Standby flag is set
  		  {
  			  // Clear the Standby flag
  			  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

  			  // Check if RTC Alarm A triggered the wake-up
  			  if (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != RESET)
  			  {
  				  // Clear the RTC Alarm A flag
  				  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

  				  // Perform wake-up logic
  				  wake_up_from_standby_handler();
  			  }
  		  }
  	#endif

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  float delta = 100 / (EARTH_HUM_DRY_VAL -  EARTH_HUM_WET_VAL); // Precalculated value to decrease power consumption
  uint8_t status = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* MAIN STATE MACHINE */
	  switch(state)
	  {

	  case STATE_INIT:

		  ////////* DEVICE BOOT *////////

		  if (measurements.ADC_read_cnt == 0)
		  {
			  /* BME280 */
			  status += BME280_Reset(&bme280, &hi2c2);
			  HAL_Delay(200);
			  status += BME280_ReadDeviceID(&bme280, &hi2c2);
			  status += BME280_ReadCalibData(&bme280, &hi2c2);
			  status += BME280_Init(&bme280, &hi2c2);

			  // Enable power to humidity probe and to voltage divider for battery measurement
			  HAL_GPIO_WritePin(EHUM_PWR_GPIO_Port, EHUM_PWR_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(D_VBAT_EN_GPIO_Port, D_VBAT_EN_Pin, GPIO_PIN_SET);

			  // Disable RTC alarm for init routine
			  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
			  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

			  HAL_TIM_Base_Start_IT(&htim2);
			  HAL_TIM_Base_Start_IT(&htim3);
		  }

		  if (measurements.ADC_read_end == 1 && measurements.ADC_read == 1)
		  {
			  // Read analog values
			  measurements.bat_voltage[measurements.ADC_read_cnt] = ADC_Read_Battery(&measurements.ADC_values[0]);
			  measurements.earth_hum[measurements.ADC_read_cnt] = ADC_Read_EHum(&measurements.ADC_values[1], &delta);
			  measurements.ADC_read_cnt ++;

			  measurements.ADC_read_end = 0; 	// reset ADC read flag
			  measurements.ADC_read = 0; 		// reset timer read flag
			  HAL_TIM_Base_Start_IT(&htim2);

			  // Calculate average values
			  if (measurements.ADC_read_cnt == 5)
			  {
				  measurements.ADC_read_cnt = 0;
				  HAL_TIM_Base_Stop_IT(&htim2);

				  // Disable power to humidity probe and to voltage divider for battery measurement
				  HAL_GPIO_WritePin(EHUM_PWR_GPIO_Port, EHUM_PWR_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(D_VBAT_EN_GPIO_Port, D_VBAT_EN_Pin, GPIO_PIN_RESET);

				  // Calculate average battery voltage
				  measurements.battery_voltage = 0;
				  measurements.earth_humidity = 0;

				  for (int i = 0; i < 5; i++)
				  {
					  measurements.battery_voltage += measurements.bat_voltage[i];
					  measurements.earth_humidity += measurements.earth_hum[i];
				  }

				  measurements.battery_voltage /= 5;
				  measurements.earth_humidity /= 5;

				  // Read all data from BME280 sensor
				  BME280_GoToFromSleep(&bme280, &hi2c2, 1); // wake-up
				  BME280_ReadAllData(&bme280, &hi2c2);
				  BME280_GoToFromSleep(&bme280, &hi2c2, 0); // sleep

				  // LoRa module
				  RFM95W_Struct_Init(&rfm95_handle);

				  // Initialise RFM95 module.
				  if (!rfm95_init(&rfm95_handle)) status++;

				  // All that flags must be cleard to get stable boot
				  NVIC_ClearPendingIRQ(EXTI1_IRQn); // Clear EXTI1 NVIC pending flag
				  NVIC_ClearPendingIRQ(EXTI3_IRQn); // Clear EXTI3 NVIC pending
				  NVIC_ClearPendingIRQ(EXTI15_10_IRQn); // Clear EXTI15_10 NVIC pending flag
				  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
				  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
				  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Bricks the program, inturrupt is not needed -> only for RX

				  // Check if everthing is OK (voltage and sensors)
				  if (measurements.battery_voltage < 2.8f) status++; //Vbat NEEDS TO BE SET

				  lora_data.errSendCnt += status;

				  if (status == 0)
				  {
					  //state = STATE_FIRST_CONN;
					  state = STATE_SEND;
					  status = 0;
					  lora_data.errSendCnt = 0;
				  }
				  else
				  {
					  //state = STATE_ERROR;
					  state = STATE_GO_SLEEP;
				  }
			  }
		  }

		  break;

	  case STATE_FIRST_CONN:

		  ////////* FIRST LoRa/SIM CONNECTION TEST *////////

		  // Create data packet that will be send - dummy
		  uint8_t test_data_packet[] = {0x01, 0x02, 0x03, 0x04};

		  // Read number of TX packets from flash
		  uint32_t tx_count = 0;
		  Flash_Read_Data(FLASH_START_ADDR, &tx_count, 1); // 1 = one word
		  rfm95_handle.config.tx_frame_count = tx_count;

		  if (!rfm95_send_receive_cycle(&rfm95_handle, test_data_packet, sizeof(test_data_packet)))
		  {
			  // Put device in error state
			  state = STATE_ERROR;
		  }
		  else
		  {
			  // Write number of TX packets to flash
			  uint32_t temp_data = (uint32_t)rfm95_handle.config.tx_frame_count;
			  Flash_Write_Data(FLASH_START_ADDR, &temp_data, 1); // 1 = one word

			  // Put device in sleep
			  state = STATE_GO_SLEEP;

			  // Send data packet
			  //state = STATE_SEND;
		  }

		  break;

	  case STATE_RUN:

		  ////////* DEVICE WAKEUP ROUTINE *////////

		  // Start all clocks
		  if (awake == 1)
		  {
			  awake = 0;
			  SystemClock_Config();
			  HAL_ResumeTick();

			  // Reset wake up flag
			  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
			  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

			  HAL_TIM_Base_Init(&htim2);
			  HAL_TIM_Base_Init(&htim3);
			  HAL_NVIC_EnableIRQ(TIM2_IRQn); // Disable Timer 2 interrupt
			  HAL_NVIC_EnableIRQ(TIM3_IRQn); // Disable Timer 3 interrupt

			  // Enable power to humidity probe and to voltage divider for battery measurement
			  HAL_GPIO_WritePin(EHUM_PWR_GPIO_Port, EHUM_PWR_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(D_VBAT_EN_GPIO_Port, D_VBAT_EN_Pin, GPIO_PIN_SET);

			  HAL_ADC_Init(&hadc);
			  HAL_ADC_Start(&hadc);
  //			  __HAL_RCC_ADC1_CLK_ENABLE();   // Enable ADC1 clock
			  HAL_I2C_Init(&hi2c1);
  //			  __HAL_RCC_I2C1_CLK_ENABLE();   // Enable I2C1 clock
			  HAL_I2C_Init(&hi2c2);
  //			  __HAL_RCC_I2C2_CLK_ENABLE();   // Enable I2C2 clock
			  HAL_SPI_Init(&hspi1);
  //			  __HAL_RCC_SPI1_CLK_ENABLE();   // Enable SPI1 clock
			  HAL_SPI_Init(&hspi2);
  //			  __HAL_RCC_SPI2_CLK_ENABLE();   // Enable SPI1 clock
			  HAL_UART_Init(&huart1);
  //			  __HAL_RCC_USART1_CLK_ENABLE(); // Enable USART1 clock

  //			  __HAL_RCC_GPIOA_CLK_ENABLE();
  //			  __HAL_RCC_GPIOB_CLK_ENABLE();
  //			  __HAL_RCC_GPIOC_CLK_ENABLE();
  //			  __HAL_RCC_GPIOD_CLK_ENABLE();

		  }

		  if (measurements.ADC_read_cnt == 0)
		  {
			  /* BME280 */
			#ifndef SLEEP_MODE_STOP
			  status += BME280_Reset(&bme280, &hi2c2);
			  HAL_Delay(200);
			  status += BME280_ReadDeviceID(&bme280, &hi2c2);
			  status += BME280_ReadCalibData(&bme280, &hi2c2);
			  status += BME280_Init(&bme280, &hi2c2);
			#endif

			  HAL_TIM_Base_Start_IT(&htim2);
			  HAL_TIM_Base_Start_IT(&htim3);
		  }

		  if (measurements.ADC_read_end == 1 && measurements.ADC_read == 1)
		  {
			  // Read analog values
			  measurements.bat_voltage[measurements.ADC_read_cnt] = ADC_Read_Battery(&measurements.ADC_values[0]);
			  measurements.earth_hum[measurements.ADC_read_cnt] = ADC_Read_EHum(&measurements.ADC_values[1], &delta);
			  measurements.ADC_read_cnt ++;

			  measurements.ADC_read_end = 0; 		// reset ADC read flag
			  measurements.ADC_read = 0; 			// reset timer read flag
			  HAL_TIM_Base_Start_IT(&htim2);

			  // Calculate average values
			  if (measurements.ADC_read_cnt == 5)
			  {
				  measurements.ADC_read_cnt = 0;
				  HAL_TIM_Base_Stop_IT(&htim2);

				  // Disable power to humidity probe and to voltage divider for battery measurement
				  HAL_GPIO_WritePin(EHUM_PWR_GPIO_Port, EHUM_PWR_Pin, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(D_VBAT_EN_GPIO_Port, D_VBAT_EN_Pin, GPIO_PIN_RESET);

				  // Calculate average battery voltage
				  measurements.battery_voltage = 0;
				  measurements.earth_humidity = 0;

				  for (int i = 0; i < 5; i++)
				  {
					  measurements.battery_voltage += measurements.bat_voltage[i];
					  measurements.earth_humidity += measurements.earth_hum[i];
				  }

				  measurements.battery_voltage /= 5;
				  measurements.earth_humidity /= 5;

				  // Read all data from BME280 sensor
				  BME280_GoToFromSleep(&bme280, &hi2c2, 1); // wake-up
				  BME280_ReadAllData(&bme280, &hi2c2);
				  BME280_GoToFromSleep(&bme280, &hi2c2, 0); // sleep

				  // LoRa module
				#ifndef SLEEP_MODE_STOP
				  RFM95W_Struct_Init(&rfm95_handle);

				  // Initialise RFM95 module.
				  if (!rfm95_init(&rfm95_handle)) status += 10; // Increase error counter for 10 to detect LoRa error (data can't be send)

				  // All that flags must be cleard to get stable boot
				  NVIC_ClearPendingIRQ(EXTI1_IRQn); // Clear EXTI1 NVIC pending flag
				  NVIC_ClearPendingIRQ(EXTI3_IRQn); // Clear EXTI3 NVIC pending
				  NVIC_ClearPendingIRQ(EXTI15_10_IRQn); // Clear EXTI15_10 NVIC pending flag
				  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
				  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
				  //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); // Bricks the program, inturrupt is not needed -> only for RX
				#endif

				  // Check if everthing is OK (voltage and sensors)
				  if (measurements.battery_voltage < 2.8f) status++; //Vbat NEED TO BE SET

				  lora_data.errSendCnt += status;

				  if (status < 10)
				  {
					  state = STATE_SEND;
				  }
				  else
				  {
					  state = STATE_ERROR;
				  }
			  }
		  }

		  break;

	  case STATE_SEND:

		  ////////* SEND DATA *////////

		  // Collect and change data for sending
		  if (lora_data.errSendCnt > 0) lora_data.error = 1;					// Flag if device is in error
		  lora_data.battery = (uint8_t)(measurements.battery_voltage * 10);		// Battery voltage [3.5V -> 35V, no float]
		  lora_data.air_temperature = (int32_t)(bme280.Temp_C);					// Air temperature [test for negative value]
		  lora_data.air_humidity = (uint8_t)(bme280.Hum_Perc);					// Air humidity in perscents [0-100%]
		  lora_data.air_pressure = (uint32_t)(bme280.Press_Pa);					// Air pressure [saved in two uint8_ts]
		  lora_data.earth_humudity = (uint8_t)(measurements.earth_humidity);	// Humidity value of earth in percents [0-100%]

		  // Data packet that will be send (modify if needed)
		  uint8_t data_packet[14];

		  data_packet[0] = DEVICE_ID;                              // Device ID number
		  data_packet[1] = lora_data.error;                        // Error flag
		  data_packet[2] = lora_data.errSendCnt;                   // No. of errors
		  data_packet[3] = lora_data.battery;                      // battery voltage

		  // Encode air_temperature (int32_t -> 4 bytes)
		  data_packet[4] = (uint8_t)(lora_data.air_temperature & 0xFF);
		  data_packet[5] = (uint8_t)((lora_data.air_temperature >> 8) & 0xFF);
		  data_packet[6] = (uint8_t)((lora_data.air_temperature >> 16) & 0xFF);
		  data_packet[7] = (uint8_t)((lora_data.air_temperature >> 24) & 0xFF);

		  // Encode air_humidity
		  data_packet[8] = lora_data.air_humidity;

		  // Encode air_pressure (uint32_t -> 4 bytes)
		  data_packet[9] = (uint8_t)(lora_data.air_pressure & 0xFF);
		  data_packet[10] = (uint8_t)((lora_data.air_pressure >> 8) & 0xFF);
		  data_packet[11] = (uint8_t)((lora_data.air_pressure >> 16) & 0xFF);
		  data_packet[12] = (uint8_t)((lora_data.air_pressure >> 24) & 0xFF);

		  // Encode earth_humidity
		  data_packet[13] = lora_data.earth_humudity;

		  // Read number of TX packets from flash
		  Flash_Read_Data(FLASH_START_ADDR, &tx_count, 1); // 1 = one word
		  rfm95_handle.config.tx_frame_count = tx_count;

		  if (!rfm95_send_receive_cycle(&rfm95_handle, data_packet, sizeof(data_packet))) // test_data_packet
		  {
			  lora_data.errSendCnt++; // Not used
		  }
		  else
		  {
			  // Write number of TX packets to flash
			  uint32_t temp_data = (uint32_t)rfm95_handle.config.tx_frame_count;
			  Flash_Write_Data(FLASH_START_ADDR, &temp_data, 1); // 1 = one word
		  }

		  state = STATE_GO_SLEEP;

		  // Reset error counter
		  status = 0;
		  lora_data.errSendCnt = 0;

		  break;

	  case STATE_GO_SLEEP:

		  ////////* PUT DEVICE TO SLEEP *////////

		  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

		  // Based on defines select sleep period
		#ifdef SLEEP_PERIOD_ONE_MINUTE
		  time.Minutes += 1;
		#endif

		#ifdef SLEEP_PERIOD_15_MINUTES
		  time.Minutes += 15;
		#endif

		#ifdef SLEEP_PERIOD_HALF_HOUR
		  time.Minutes += 30;
		#endif

		#ifdef SLEEP_PERIOD_ONE_HOUR
		  time.Hours += 1;
		#endif

		#ifdef SLEEP_PERIOD_CUSTOM
		  time.Minutes += SLEEP_PERIOD_CUSTOM;
		#endif

		  if(time.Seconds>=60)
		  {
			  time.Minutes ++;
			  time.Seconds = time.Seconds - 60;
		  }

		  if(time.Minutes >= 60)
		  {
			  time.Hours++;
			  time.Minutes = time.Minutes - 60;
		  }

		  if(time.Hours > 23)
		  {
			  time.Hours = 0;
		  }

		  sAlarm.Alarm = RTC_ALARM_A;
		  sAlarm.AlarmTime.Hours = time.Hours;
		  sAlarm.AlarmTime.Minutes = time.Minutes;
		  sAlarm.AlarmTime.Seconds = time.Seconds;
		  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN);

		  // Stop Timers if they running and disable their interrupts
		  HAL_TIM_Base_Stop_IT(&htim2); // Stop Timer 2
		  HAL_TIM_Base_Stop_IT(&htim3); // Stop Timer 3
		  HAL_NVIC_DisableIRQ(TIM2_IRQn); // Disable Timer 2 interrupt
		  HAL_NVIC_DisableIRQ(TIM3_IRQn); // Disable Timer 3 interrupt


		  HAL_ADC_Stop(&hadc);
		  HAL_ADC_DeInit(&hadc);
  ////		  __HAL_RCC_ADC1_CLK_DISABLE();   // Disable ADC1 clock
  //		  HAL_I2C_DeInit(&hi2c1);
  ////		  __HAL_RCC_I2C1_CLK_DISABLE();   // Disable I2C1 clock
  //		  HAL_I2C_DeInit(&hi2c2);
  ////		  __HAL_RCC_I2C2_CLK_DISABLE();   // Disable I2C2 clock
  //		  HAL_SPI_DeInit(&hspi1);
  ////		  __HAL_RCC_SPI1_CLK_DISABLE();   // Disable SPI1 clock
  //		  HAL_SPI_DeInit(&hspi2);
  ////		  __HAL_RCC_SPI2_CLK_DISABLE();   // Disable SPI2 clock
  //		  HAL_UART_DeInit(&huart1);
  //		  __HAL_RCC_USART1_CLK_DISABLE(); 	  // Disable USART1 clock
  //
  //		  __HAL_RCC_GPIOA_CLK_DISABLE();
  //		  __HAL_RCC_GPIOB_CLK_DISABLE();
  //		  __HAL_RCC_GPIOC_CLK_DISABLE();
  //		  __HAL_RCC_GPIOD_CLK_DISABLE();



  //		  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  //		  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  //		  RCC_OscInitStruct.LSIState = RCC_LSI_OFF; // Disable LSI
  //		  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  //
  //		  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  //		  RCC_OscInitStruct.HSIState = RCC_HSI_OFF; // Disable HSI
  //		  HAL_RCC_OscConfig(&RCC_OscInitStruct);


		  rfm95_goto_sleep(&rfm95_handle); // If the module is not in sleep mode (it should be)


		  HAL_SuspendTick();

		  // Set sleep mode
		#ifdef SLEEP_MODE_STOP
		  HAL_DBGMCU_DisableDBGStopMode();
		  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); // PWR_MAINREGULATOR_ON
		#endif

		#ifndef SLEEP_MODE_STOP
		  HAL_DBGMCU_DisableDBGStandbyMode();
		  HAL_PWR_EnterSTANDBYMode();
		#endif

		  // Resume clock and ticks
		  SystemClock_Config();
		  HAL_ResumeTick();

		  break;

	  case STATE_ERROR:

		  ////////* INIT ROUTINE FAILED *////////

		  break;

	  default:

		  ////////* UNDEFINED STATE *////////

		  break;

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BOOST_EN_Pin|EHUM_PWR_Pin|SPI_CS_EX_Pin|RESET_Pin
                          |SIM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D_VBAT_EN_Pin|SPI2_NSS_Pin|SIM_SLP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BOOST_EN_Pin EHUM_PWR_Pin SPI_CS_EX_Pin RESET_Pin
                           SIM_RST_Pin */
  GPIO_InitStruct.Pin = BOOST_EN_Pin|EHUM_PWR_Pin|SPI_CS_EX_Pin|RESET_Pin
                          |SIM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D_VBAT_EN_Pin SPI2_NSS_Pin SIM_SLP_Pin */
  GPIO_InitStruct.Pin = D_VBAT_EN_Pin|SPI2_NSS_Pin|SIM_SLP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO5_Pin DIO0_Pin SIM_ISR_Pin */
  GPIO_InitStruct.Pin = DIO5_Pin|DIO0_Pin|SIM_ISR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UNUSED1_Pin */
  GPIO_InitStruct.Pin = UNUSED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UNUSED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO2_Pin */
  GPIO_InitStruct.Pin = DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO4_Pin DIO3_Pin */
  GPIO_InitStruct.Pin = DIO4_Pin|DIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  //HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  //HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  // ^^^^^^^^ COMMENT ALL IRQ ENABLE WRITEN BY IDE ^^^^^^^^
  // Manual setup IRQs - interrupt signal already at startup
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 1); // 1,0
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Wake up interrupt - STOP mode //
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	awake = 1;
	if (state != STATE_INIT) state = STATE_RUN;
}


// Wake up interrupt - STANDBY mode //
void wake_up_from_standby_handler(void)
{
    awake = 1;
	state = STATE_RUN;
}


// TIMER Interrupt //
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{
	/* TIMER 2 - 10Hz (Read sensors to average their values) */
	if (htim->Instance == TIM2)
	{
		if (measurements.ADC_read == 0)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_ADC_Start_IT(&hadc);
			measurements.ADC_read = 1;
		}
	}

	if (htim->Instance == TIM3)
	{
		tim3_tick_msb += 0x10000; // Increment the MSB by 0x10000 each time TIM2 overflows
		HAL_TIM_Base_Start_IT(&htim3);
	}
}


// ADC interrupt
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
    	measurements.ADC_values[0] = HAL_ADC_GetValue(hadc);
    	measurements.ADC_values[1] = HAL_ADC_GetValue(hadc);
    	HAL_ADC_Stop_IT(hadc);
    	measurements.ADC_read_end = 1;
    }
}


// Read battery voltage //
float ADC_Read_Battery(uint32_t* ADC_value)
{
	float Vout = ((float)(*ADC_value * LDO_OUT_U)) / 4095;
	float voltage = ((BAT_R1 + BAT_R2) * (Vout / BAT_R2));
	return (float) voltage;
}


// EFM95W module //
void RFM95W_Struct_Init(rfm95_handle_t* rfm95_handle)
{
		rfm95_handle->spi_handle = &hspi2;
		rfm95_handle->nss_port = SPI2_NSS_GPIO_Port;
		rfm95_handle->nss_pin = SPI2_NSS_Pin;
		rfm95_handle->nrst_port = RESET_GPIO_Port;
		rfm95_handle->nrst_pin = RESET_Pin;

		rfm95_handle->precision_tick_frequency = 32768;
		rfm95_handle->precision_tick_drift_ns_per_s = 5000;
		rfm95_handle->receive_mode = RFM95_RECEIVE_MODE_NONE;
		rfm95_handle->get_precision_tick = get_precision_tick;
		//rfm95_handle->precision_sleep_until = precision_sleep_until;
		rfm95_handle->random_int = random_int;

		uint8_t address[] = DEVICE_ADDRESS;
		memcpy(rfm95_handle->device_address, address, sizeof(address));

		uint8_t sKey[] = APPSKEY;
		memcpy(rfm95_handle->application_session_key, sKey, sizeof(sKey));

		uint8_t nKey[] = NWKSKEY;
		memcpy(rfm95_handle->network_session_key, nKey, sizeof(nKey));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//__disable_irq();
    if (GPIO_Pin == DIO0_Pin) {
        rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO0);
    } else if (GPIO_Pin == DIO1_Pin) {
        rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO1);
    } else if (GPIO_Pin == DIO5_Pin) {
        rfm95_on_interrupt(&rfm95_handle, RFM95_INTERRUPT_DIO5);
    } else {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin); // Clear any unexpected interrupt
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin); // Clear the flag
    //__enable_irq();
}

static uint8_t random_int(uint8_t max)
{
    return (uint8_t)(measurements.ADC_values[0] & 0x000f); // Use ADC other means of obtaining a random number.
}

static uint32_t get_precision_tick()
{
    __disable_irq(); // Disable interrupts to ensure atomic access to tick variables
    uint32_t precision_tick = tim3_tick_msb | __HAL_TIM_GET_COUNTER(&htim3);
    __enable_irq(); // Re-enable interrupts
    return precision_tick;
}

// Read humidity in ground //
uint8_t ADC_Read_EHum(uint32_t* ADC_value, float* delta)
{
	uint8_t humidity = (uint8_t)round(*ADC_value - EARTH_HUM_WET_VAL) * (*delta);
	return humidity;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
