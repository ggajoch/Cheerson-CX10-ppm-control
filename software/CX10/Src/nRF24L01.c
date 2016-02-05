#include "spi.h"
#include "nRF24L01.h"
static uint8_t rf_setup;


void delayMicroseconds(uint32_t i) {
	if( i > 1000 ) {
		HAL_Delay(i/1000 + 1);
	} else {
		volatile int16_t delay = i*50;
		while(delay--);
	}
}

uint8_t NRF24L01_WriteReg(uint8_t address, uint8_t data)
{
    CS_off;
    _spi_write_address(address | W_REGISTER, data);
    CS_on;
    return 1;
}

void NRF24L01_WriteRegisterMulti(uint8_t address, const uint8_t data[], uint8_t len)
{
    delayMicroseconds(5);
    CS_off;
    _spi_write(address | W_REGISTER);
    uint8_t i;
    for(i = 0; i < len;i++)
        _spi_write(data[i]);
    CS_on;
    delayMicroseconds(5);
}

void NRF24L01_Initialize()
{
    rf_setup = 0x0F;
}

static uint8_t Strobe(uint8_t state)
{
    uint8_t result;
    CS_off;
    result = _spi_write(state);
    CS_on;
    return result;
}

uint8_t NRF24L01_FlushTx()
{
    return Strobe(FLUSH_TX);
}

uint8_t NRF24L01_FlushRx()
{
    return Strobe(FLUSH_RX);
}

uint8_t NRF24L01_WritePayload(uint8_t *data, uint8_t length)
{
    CE_off;
    CS_off;
    _spi_write(W_TX_PAYLOAD);
    uint8_t i;
    for(i = 0; i<length; i++)
        _spi_write(data[i]);
    CS_on;
    CE_on; // transmit
    return 1;
}

uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
{
    uint8_t i;
    CS_off;
    _spi_write(R_RX_PAYLOAD); // Read RX payload
    for (i=0;i<length;i++) {
        data[i]=_spi_read();
    }
    CS_on;
    return 1;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    CS_off;
    uint8_t data = _spi_read_address(reg);
    CS_on;
    return data;
}

uint8_t NRF24L01_Activate(uint8_t code)
{
    CS_off;
    _spi_write(ACTIVATE);
    _spi_write(code);
    CS_on;
    return 1;
}

void NRF24L01_SetTxRxMode(uint8_t mode)
{
    if(mode == TX_EN) {
        CE_off;
        NRF24L01_WriteReg(NRF24L01_07_STATUS,
                    (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                  | (1 << NRF24L01_07_TX_DS)
                  | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                    (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                  | (1 << NRF24L01_00_CRCO)
                  | (1 << NRF24L01_00_PWR_UP));
        delayMicroseconds(130);
        CE_on;
    } else if (mode == RX_EN) {
        CE_off;
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
        NRF24L01_WriteReg(NRF24L01_07_STATUS,
                    (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                  | (1 << NRF24L01_07_TX_DS)
                  | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                    (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                  | (1 << NRF24L01_00_CRCO)
                  | (1 << NRF24L01_00_PWR_UP)
                  | (1 << NRF24L01_00_PRIM_RX));
        delayMicroseconds(130);
        CE_on;
    } else {
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); //PowerDown
        CE_off;
    }
}

uint8_t NRF24L01_Reset()
{
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    uint8_t status1 = Strobe(0xFF); // NOP
    static char buf[20];
    uint8_t status2 = NRF24L01_ReadReg(0x07);
    NRF24L01_SetTxRxMode(TXRX_OFF);
    return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}

// Power is in range 0..3 for nRF24L01
uint8_t NRF24L01_SetPower(uint8_t power)
{
    rf_setup = (rf_setup & 0xF9) | ((power & 0x03) << 1);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

uint8_t NRF24L01_SetBitrate(uint8_t bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

