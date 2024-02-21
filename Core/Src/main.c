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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void SystemClock_Config(void);

void transmitChar(char c) {
	// continue looping if transmit data register is empty
	if ( (USART3->ISR & 128) == 0) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	}
	
	while ( (USART3->ISR & 128) == 0) {
		
	}
	
	
	USART3->TDR = c;
	return;
}

void transmitString(char string[]) {
		int i = 0;
		while (string[i] != 0) {
			transmitChar(string[i]);
			i++;
		}
		return;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	// Set up a configuration of LEDs
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC6, PC7, PC8 & PC9
	
	// enable system clock as USART3 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// USART3_TX = PC4
	// USART3_RX = PC5
	
	// USB-UART Transmit 	(TX) -> STM32F0 Receive 	(RX)
	// USB-UART Receive 	(RX) -> STM32F0 Transmit 	(TX)
	
	// setting to Alternate Function Mode
	// PC4
	GPIOC->MODER |= (1 << 9);
	GPIOC->MODER &= ~(1 << 8);
	
	// PC5
	GPIOC->MODER |= (1 << 11);
	GPIOC->MODER &= ~(1 << 10);

	// selecting alternate functions AF1
	// PC4
	GPIOC->AFR[0] &= ~(1 << 19);
	GPIOC->AFR[0] &= ~(1 << 18);
	GPIOC->AFR[0] &= ~(1 << 17);
	GPIOC->AFR[0] |= (1 << 16);
	
	// PC5
	GPIOC->AFR[0] &= ~(1 << 23);
	GPIOC->AFR[0] &= ~(1 << 22);
	GPIOC->AFR[0] &= ~(1 << 21);
	GPIOC->AFR[0] |= (1 << 20);
	
	// setting Baud rate to be 115200
	// USART3->BRR &= ~(1 << 20);	// disable auto buad rate
	
	USART3->CR1 |= (1 << 5);
	
	USART3->CR1 |= (1 << 3);	// enable TX
	USART3->CR1 |= (1 << 2);  // enable RX
	
	/*
	int f = HAL_RCC_GetHCLKFreq();
	int target = 115200;
	int brr_val = f / target;
	*/
	USART3->BRR = 8000000 / 115200 ;
	
	USART3->CR1 |= (1 << 0);		// enable USART
	
  while (1)
  {
		HAL_Delay(200);

			
		while ( (USART3->ISR & 32) == 0) {
			
		}
		
		switch (USART3->RDR) {
			case 'r':
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
				break;
			case 'o':
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
				break;
			case 'b':
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
				break;
			case 'g':
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
				break;
			default:
				transmitString("Unknown Command...\0");
		}
		
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
