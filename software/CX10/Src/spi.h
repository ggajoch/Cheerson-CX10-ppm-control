///////////////
// SPI

#include "stm32f0xx_hal.h"

extern SPI_HandleTypeDef hspi1;


#define CS_off HAL_GPIO_WritePin(nRF_CSN_GPIO_Port, nRF_CSN_Pin, DISABLE)
#define CS_on HAL_GPIO_WritePin(nRF_CSN_GPIO_Port, nRF_CSN_Pin, ENABLE)

#define CE_off HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, DISABLE)
#define CE_on HAL_GPIO_WritePin(nRF_CE_GPIO_Port, nRF_CE_Pin, ENABLE)

#define _NOP() __asm__ __volatile__("nop")


uint8_t _spi_write(uint8_t command);
void _spi_write_address(uint8_t address, uint8_t data);
uint8_t _spi_read();
uint8_t _spi_read_address(uint8_t address);
