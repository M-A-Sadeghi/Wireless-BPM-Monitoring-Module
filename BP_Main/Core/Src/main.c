/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THRESHOLD 3500   // Example threshold for peak detection
#define A 0.5            // Hypothetical calibration constant for SBP
#define B 85            // Hypothetical calibration constant for SBP
#define C 0.3            // Hypothetical calibration constant for DBP
#define D 55             // Hypothetical calibration constant for DBP

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
void ESP_SendCommand(char *command);
uint8_t RxBuffer[1024];
uint8_t TxBuffer[1024];
uint8_t danger = 0;
int sbp_int = 0;
int dbp_int = 0;
uint8_t ttt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    HAL_UART_Receive_IT(huart, RxBuffer, sizeof(RxBuffer));
}

void ESP_SendCommand(char *command) {
    sprintf((char *)TxBuffer, "%s", command);
    HAL_UART_Transmit(&huart1, TxBuffer, strlen((char *)TxBuffer), 1000);
//    HAL_UART_Receive(&huart1, RxBuffer, sizeof(RxBuffer), 1000);  // Receive response (optional)
//    HAL_UART_DeInit(&huart1);
//    HAL_UART_Init(&huart1);

}
// Function to read ADC value
uint32_t Read_ADC(void) {
    HAL_ADC_Start(&hadc);                          // Start ADC conversion
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY); // Wait for conversion
    uint32_t adc_value = HAL_ADC_GetValue(&hadc);  // Get ADC value
    HAL_ADC_Stop(&hadc);                           // Stop ADC conversion
    return adc_value;
}

// Function to send data via UART
//void UART_Send(char *message) {
//    HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
//}

// Function to estimate and send BP
void CalculateAndSendBP(uint32_t hr) {
    float sbp = A * hr + B;
    float dbp = C * hr + D;

    sbp_int = (int)sbp;  // Convert to integer for display
	dbp_int = (int)dbp;  // Convert to integer for display

//    HAL_UART_Transmit(&huart1, (uint8_t *) &sbp, 4, 1000);
//    HAL_UART_Transmit(&huart1, (uint8_t *) &dbp, 4, 1000);

    if (hr>103 || sbp > 145) {
		// Send HTTP response
		danger = 1;
	}
    else
    {
    	danger = 0;
    }



//    char buffer[100];
//    sprintf(buffer, "HR: %lu BPM, SBP: %.2f mmHg, DBP: %.2f mmHg\r\n", hr, sbp, dbp);
//    UART_Send(buffer);
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint32_t previous_peak_time = 0;
  uint32_t current_peak_time = 0;
  uint32_t adc_value = 0;
  uint32_t hr = 0;
  uint32_t time_diff = 0;

  // Initialize ESP8266 with AT commands
//      ESP_SendCommand("AT+RESTORE\r\n");         // Test if ESP is working
//      HAL_Delay(1000);
      ESP_SendCommand("AT+RST\r\n");         // Test if ESP is working
      HAL_Delay(2000);
      ESP_SendCommand("AT+CWMODE=2\r\n");  // Set ESP8266 to AP mode (CWMODE=2)
	  HAL_Delay(2000);

	  ESP_SendCommand("AT+CWSAP=\"Zaraban\",\"12345678\",5,3\r\n");  // Set SSID and password (Max 8 chars for WPA2)
	  HAL_Delay(2000);

	  ESP_SendCommand("AT+CIPMUX=1\r\n");  // Enable multiple connections
	  HAL_Delay(1000);

	  ESP_SendCommand("AT+CIPSERVER=1,80\r\n");  // Start the server on port 80
	  HAL_Delay(1000);
//      HAL_Delay(5000);

      // Set up the web server
//      ESP_SendCommand("AT+CIPMUX=1\r\n");  // Enable multiple connections
//      HAL_Delay(1000);
//      ESP_SendCommand("AT+CIPSERVER=1,80\r\n");  // Start the server on port 80
//      HAL_Delay(1000);

//	  ESP_SendCommand("AT+CIFSR\r\n");

	  HAL_UART_Receive_DMA(&huart1, RxBuffer, sizeof(RxBuffer));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ttt++;
 	  if (strstr((char *)RxBuffer, "+IPD") && ttt % 2 == 1) {
 		 char html_page[512];

 		if (danger == 1) {
		  sprintf(html_page,
				  "HTTP/1.1 200 OK\r\n"
				  "Content-Type: text/html\r\n\r\n"
				  "<html>"
				  "<head><meta http-equiv=\"refresh\" content=\"3\"></head>"
				  "<body>"
				  "<h1>Alarm!!!</h1>"
//				  "<p>HR: %lu BPM</p>"
				  "<p>SBP: %d mmHg</p>"
				  "<p>DBP: %d mmHg</p>"
				  "</body>"
				  "</html>",
				  sbp_int, dbp_int);
	  } else {
		  sprintf(html_page,
				  "HTTP/1.1 200 OK\r\n"
				  "Content-Type: text/html\r\n\r\n"
				  "<html>"
				  "<head><meta http-equiv=\"refresh\" content=\"3\"></head>"
				  "<body>"
				  "<h1>Normal!</h1>"
//				  "<p>HR: %lu BPM</p>"
				  "<p>SBP: %d mmHg</p>"
				  "<p>DBP: %d mmHg</p>"
				  "</body>"
				  "</html>",
				   sbp_int, dbp_int);
	  }

//		  char *html_page =
//		                  "HTTP/1.1 200 OK\r\n"
//		                  "Content-Type: text/html\r\n\r\n"
//		                  "<html>"
//		                  "<head><meta http-equiv=\"refresh\" content=\"1\"></head>"
//		                  "<body>"
//		                  "<h1>Normal!</h1>"
//		                  "</body>"
//		                  "</html>";
//
//		  char *html_page2 =
//		  		                  "HTTP/1.1 200 OK\r\n"
//		  		                  "Content-Type: text/html\r\n\r\n"
//		  		                  "<html>"
//		  		                  "<head><meta http-equiv=\"refresh\" content=\"1\"></head>"
//		  		                  "<body>"
//		  		                  "<h1>Alarm!!!</h1>"
//		  		                  "</body>"
//		  		                  "</html>";

		              // Send HTML response
//		              ESP_SendCommand("AT+CIPSEND=0,128\r\n");  // Adjust length based on your HTML content
 					  ESP_SendCommand("AT+CIPSEND=0,256\r\n");
 					  HAL_Delay(100);
// 					 memset(RxBuffer, 0, sizeof(RxBuffer));
		              ESP_SendCommand(html_page);

//		              HAL_Delay(100);
//		              HAL_UART_AbortReceive_IT(&huart1);
//		              for(int i = 0; i < 200; i++) RxBuffer[i] = 0;
//		        	  HAL_UART_Receive_IT(&huart1, RxBuffer, sizeof(RxBuffer));


		  // Close connection after sending response
		  ESP_SendCommand("AT+CIPCLOSE=5\r\n");
//          memset(RxBuffer, 0, sizeof(RxBuffer));

	  }
	  adc_value = Read_ADC();  // Read Pulse Sensor value

	  if (adc_value > THRESHOLD) {
		  // A peak is detected
		  current_peak_time = HAL_GetTick();  // Get current time in ms

		  if (previous_peak_time != 0) {
			  // Calculate heart rate (BPM)
			  time_diff = current_peak_time - previous_peak_time;
			  hr = 60000 / time_diff;  // Convert ms to BPM

			  // Estimate and send BP via UART
			  if(hr > 60 && hr < 110)
				  CalculateAndSendBP(hr);
		  }

		  previous_peak_time = current_peak_time;  // Update previous peak time

		  // Debounce to avoid multiple detections of the same peak
//		  HAL_Delay(250);  // 250ms delay
	  }

//	  HAL_Delay(10);  // Small delay to control sampling rate
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
