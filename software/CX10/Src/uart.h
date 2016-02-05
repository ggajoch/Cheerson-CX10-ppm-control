#include "stm32f0xx_hal.h"

extern UART_HandleTypeDef huart1;



void UART_TX(char * tab) {
	HAL_UART_Transmit(&huart1, (uint8_t *)tab, strlen(tab), 1000);
}
