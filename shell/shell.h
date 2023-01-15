/*
 * shell.h
 *
 *  Created on: 7 juin 2019
 *      Author: laurent
 */

#ifndef INC_LIB_SHELL_SHELL_H_
#define INC_LIB_SHELL_SHELL_H_

#include <stdint.h>
#include "cmsis_os.h"
#include "semphr.h"
#define UART_DEVICE huart1

extern SemaphoreHandle_t sem1;

#define ARGC_MAX 8
#define BUFFER_SIZE 256
#define SHELL_FUNC_LIST_MAX_SIZE 64
static char print_buffer[BUFFER_SIZE];

void shell_init();
int shell_add(char c, int (* pfunc)(int argc, char ** argv), char * description);
int shell_run();
int uart_write(char * s, uint16_t size);

#endif /* INC_LIB_SHELL_SHELL_H_ */
