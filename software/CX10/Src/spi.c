#include "spi.h"

uint8_t _spi_write(uint8_t command)
{
	uint8_t result=0;
	HAL_SPI_TransmitReceive(&hspi1, &command, &result, 1, 1000);
	return result;
}

void _spi_write_address(uint8_t address, uint8_t data)
{
    CS_off;
    _spi_write(address);
    _NOP();
    _spi_write(data);
    CS_on;
}

uint8_t _spi_read()
{
    return _spi_write(0);
}

uint8_t _spi_read_address(uint8_t address)
{
    uint8_t result;
    CS_off;
    _spi_write(address);
    result = _spi_read();
    CS_on;
    return(result);
}
