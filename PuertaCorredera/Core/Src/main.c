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
#define USS_THRESHOLD 20 // distancia en cm que tiene que detectar el sesnro de la puerta
#define USS_TIME_THRESHOLD 1000 // tiempo de espera para que la persona entre al museo

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t tiempoRef;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Delay_us(uint32_t us);
float HC_SR04_ReadDistance(void); // Función para medir la distancia del sensor

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  // Habilitar el TRC (Trace Control)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  // Reiniciar el contador de ciclos
  DWT->CYCCNT = 0;
  // Habilitar el contador
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(HC_SR04_ReadDistance() <= USS_THRESHOLD && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0 && (HAL_GetTick() - tiempoRef) >= USS_TIME_THRESHOLD){ // si se detecta a alquien en la puerta y esta está cerrada
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1){ // cuando se active el primer fin de carrera
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
			  tiempoRef = HAL_GetTick();
		  }
	  }
	  if(HC_SR04_ReadDistance() <= USS_THRESHOLD && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0 && (HAL_GetTick() - tiempoRef) >= USS_TIME_THRESHOLD){ // si se detecta a alquien en la puerta y esta está abierta
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1){ // cuando se active el primer fin de carrera
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
			  tiempoRef = HAL_GetTick();
		  }
	  }
	  else{
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 1);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /* PC0 como salida (Trig) */
  GPIO_InitStruct.Pin   = GPIO_PIN_0;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PC1 como entrada (Echo) */
  GPIO_InitStruct.Pin   = GPIO_PIN_1;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float HC_SR04_ReadDistance(void)
{
    uint32_t tiempoEcho_us = 0;
    float distancia_cm = 0.0f;

    // 1) Generar pulso de 10 us en TRIG (PC0)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    DWT_Delay_us(2);      // Pequeño retraso
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    DWT_Delay_us(10);     // Pulso de 10 us
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

    // 2) Esperar a que Echo (PC1) se ponga en alto
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET);

    // Iniciamos conteo de tiempo
    uint32_t startTick = DWT->CYCCNT;

    // 3) Esperar a que Echo se ponga en bajo
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_SET);

    // Fin de la medición
    uint32_t endTick = DWT->CYCCNT;

    // 4) Calcular duración en microsegundos
    //    Suponiendo SystemCoreClock = 168 MHz
    //    1 tick DWT ~ 1/168e6 s => us = ticks / (168e6/1e6)
    tiempoEcho_us = (uint32_t)((endTick - startTick) / (SystemCoreClock / 1000000.0f));

    // 5) Calcular distancia en cm
    //    Distancia (cm) ≈ tiempoEcho_us / 58
    distancia_cm = (float)tiempoEcho_us / 58.0f;

    return distancia_cm;
}

/**
  * @brief Retardo en microsegundos usando DWT
  * @param us: microsegundos a esperar
  */
void DWT_Delay_us(uint32_t us)
{
    // Valor inicial de conteo
    uint32_t startTick = DWT->CYCCNT;
    // Calcular cuántos ticks de CPU corresponden a "us" microsegundos
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    // Esperar hasta que se cumpla
    while ((DWT->CYCCNT - startTick) < ticks);
}

int flancoUSS(){

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
