/*
 * drv_uart1.c
 *
 *  Created on: 7 nov. 2022
 *      Author: laurentf
 */

#include "drv_uart1.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"
#include "semphr.h"

extern SemaphoreHandle_t sem1;
uint8_t drv_uart1_receive(char * pData, uint16_t size)
{
	HAL_UART_Receive_IT(&huart1, (uint8_t*)(pData), size);
	xSemaphoreTake(sem1, HAL_MAX_DELAY);
	return 0;	// Life's too short for error management
}

uint8_t drv_uart1_transmit(const char * pData, uint16_t size)
{
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)pData, size);

	return 0;	// Srsly, don't do that kids
}
