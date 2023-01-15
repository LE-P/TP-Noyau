/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "shell.h"
#include "drv_uart1.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SIZE 512
#define SHELL_PRIORITY 11
#define CLIGNOTTE_PRIORITY 9
#define TASKLED_PRIORITY 9
#define TOTAL_HEAP_SIZE 2048
#define DELAY_1 50
#define STACK_SIZE 50
#define LED_GPIO_Port GPIOI
#define LED_Pin 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TaskHandle_t ShellHandle;
TaskHandle_t ClignotteHandle;
TaskHandle_t xHandle;
char *pvParameters;
SemaphoreHandle_t sem1;
SemaphoreHandle_t semMutex;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

void TaskLed(void * pvParameters){
	int freq = (int) pvParameters; // On récupère la réquence de clignotement voulue
	while(1){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // On fait clignoter la led
		vTaskDelay(1000/freq);						// à la fréquence voulue
	}
}

/* Fonctions Shell */
int led(int argc, char ** argv){
	int f=atoi(argv[1]); // On récupère la fréquence souhaitée
	if (xHandle!=NULL ) { // Si une tache est déjà écrite on l'efface
		vTaskDelete(xHandle);
		xHandle = NULL; // On réinitialise la pile
	}
	if (f!=0){ //si la fréquence est différente de 0
		xTaskCreate(TaskLed, "TaskLed", STACK_SIZE, (void *) f, 20, &xHandle);
		// Alors on crée la tâche de clignotement de led qui a pour pvParameters la fréq f
	}
	if (f==0){ // Si la fréquence est nulle
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0); // On configure la sortie du pin à 0
	}
	else return 0;
}

int fonction(int argc, char ** argv){
	printf("Fonction bidon\r\n");

	// TODO: Ajouter l'affichage des arguments
	printf("argc = %d\r\n", argc);
	for (int i=0; i<argc; i++){
		printf("argument numero %d = %s\r\n",i, argv[i]);
	}
	return 0;
}


/* Taches OS*/

int sClignotte(int Time){
	HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_1);
	vTaskDelay(Time);
	return 1;
}

void clignotte(void * pvParameters)
{
	int Time;
	while(1)
	{
		Time = (int)pvParameters;
		sClignotte(Time);
	}
}

void CodeTache2  (void* pvParameters){
	int duree=(int) pvParameters;
	char* s=pcTaskGetName(xTaskGetCurrentTaskHandle());

	while(1){

		printf("Je suis la tache %s et je m'endors pour ùd periodes \r\n", s , duree);
	}
}

void vTask1(void* pvParameters){
	int delay = (int) pvParameters;

	while(1) {
		xSemaphoreTake(semMutex, portMAX_DELAY);
		printf("Je suis la tache 1 et je m'endors pour %d ticks\r\n", delay);
		xSemaphoreGive(semMutex);
		vTaskDelay(delay);
	}
}

void vTask2(void * pvParameters){
	int delay = (int) pvParameters;

	while(1) {
		xSemaphoreTake(semMutex, portMAX_DELAY);
		printf("Je suis la tache 2 et je m'endors pour %d ticks\r\n", delay);
		xSemaphoreGive(semMutex);
		vTaskDelay(delay);
	}
}


void codeTache1(void* pvParameters){
	while(1){
		xSemaphoreTake(sem1,portMAX_DELAY);
		printf("Tache 1 \r\n");
	}
	}

void codeTache2(void* pvParameters){
	while(1){
		xSemaphoreGive(sem1);
		printf("Tache 2 \r\n");
		vTaskDelay(DELAY_1);
	}
}

int shell(void * pvParameters)
{
	shell_add('f', fonction, "Une fonction inutile");
	shell_add('l', led, "Fait clignoter la LED");
	while(1)
	{
		shell_run();
	}
	return 0;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	sem1 = xSemaphoreCreateBinary();
	semMutex = xSemaphoreCreateMutex();
	shell_init();

	/* Creation Taches */

	xTaskCreate(shell,
				"shell",
		        MAX_SIZE,
		        *pvParameters,
		        SHELL_PRIORITY,
		        &ShellHandle
				);

	xTaskCreate(TaskLed,
				"TaskLed",
		        MAX_SIZE,
		        *pvParameters,
		        TASKLED_PRIORITY,
		        &ShellHandle
				);

	xTaskCreate(clignotte,
		        "clignotte",
		        MAX_SIZE,
		        *pvParameters,
		        CLIGNOTTE_PRIORITY,
		        &ClignotteHandle
		        );
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
