#include <stdint.h>
#include "CX10.h"
#include "nRF24L01.h"

////////////////
// Variables
static uint8_t board_type = CX10_BLUE; // board type, change as required

static uint8_t txid[4]; // transmitter ID
static uint8_t freq[4]; // frequency hopping table
static uint8_t packet[BLUE_PACKET_LENGTH];
static uint8_t packet_length;
static uint32_t packet_period;
static uint8_t current_chan = 0;

#define BV(bit) (1 << bit)

///////////////
// CX10 functions
void CX10_init()
{
    txid[1] %= 0x30;
    freq[0] = (txid[0] & 0x0F) + 0x03;
    freq[1] = (txid[0] >> 4) + 0x16;
    freq[2] = (txid[1] & 0x0F) + 0x2D;
    freq[3] = (txid[1] >> 4) + 0x40;

    if(board_type == CX10_BLUE) {
        packet_length = BLUE_PACKET_LENGTH;
        packet_period = BLUE_PACKET_PERIOD;
        uint8_t i;
        for(i = 0; i<4; i++)
            packet[5+i] = 0xff;
    }
    else if(board_type == CX10_GREEN) {
        packet_length = GREEN_PACKET_LENGTH;
        packet_period = GREEN_PACKET_PERIOD;
    }

    current_chan = 0;
    NRF24L01_Reset();
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    HAL_Delay(10);
    XN297_SetTXAddr(tx_rx_id,5);
    XN297_SetRXAddr(tx_rx_id,5);
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment on all data pipes
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, packet_length); // rx pipe 0 (used only for blue board)
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(3);                             // maximum rf power
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits on
    HAL_Delay(150);
}

void CX10_bind()
{
    uint16_t counter=BIND_COUNT;
    uint8_t bound = 0;
    uint32_t timeout;
    while(bound == 0) {
        NRF24L01_SetTxRxMode(TX_EN);
        XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, RF_BIND_CHANNEL);
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
        NRF24L01_FlushTx();
        CX10_Write_Packet(0xAA); // send bind packet
        switch(board_type) {
            case CX10_GREEN:
                if(counter == 0)
                    bound = 1;
                delayMicroseconds(packet_period);
                break;
            case CX10_BLUE:
                HAL_Delay(1);
                CE_off;
                NRF24L01_SetTxRxMode(RX_EN);
                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();
                XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP) | BV(NRF24L01_00_PRIM_RX));
                CE_on;
                volatile uint16_t i = 5000;
                while (i--) {
                    if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR)) { // data received from aircraft
                        XN297_ReadPayload(packet, packet_length);
                        if( packet[9] == 0x01)
                            bound = 1;
                        break;
                    }
                }
                break;
        }
        HAL_GPIO_TogglePin(LED_PIN_GPIO_Port, LED_PIN_Pin);
    }
    HAL_GPIO_WritePin(LED_PIN_GPIO_Port, LED_PIN_Pin, 1);
}

uint32_t process_CX10()
{
    XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, freq[current_chan++]);
    current_chan %= NUM_RF_CHANNELS;
    CX10_Write_Packet(0x55);
    return packet_period;
}


void CX10_Write_Packet(uint8_t init)
{
    uint8_t offset = 0;
    if(board_type == CX10_BLUE)
        offset = 4;
    packet[0] = init;
    packet[1] = txid[0];
    packet[2] = txid[1];
    packet[3] = txid[2];
    packet[4] = txid[3];
    // packet[5] to [8] (aircraft id) is filled during bind for blue board

    extern volatile uint16_t PPM_data[8];
#define lowByte(x) (x & 0xFF)
#define highByte(x) ((x >> 8) & 0xFF)

    packet[5+offset] = lowByte(PPM_data[AILERON]);
    packet[6+offset]= highByte(PPM_data[AILERON]);
    packet[7+offset]= lowByte(PPM_data[ELEVATOR]);
    packet[8+offset]= highByte(PPM_data[ELEVATOR]);
    packet[9+offset]= lowByte(PPM_data[THROTTLE]);
    packet[10+offset]= highByte(PPM_data[THROTTLE]);
    if(PPM_data[AUX2] > PPM_MID)
        packet[10+offset] |= 0x10; // flip flag
    packet[11+offset]= lowByte(PPM_data[RUDDER]);
    packet[12+offset]= highByte(PPM_data[RUDDER]);
    // rate / mode (use headless channel)

    if(PPM_data[AUX1] > PPM_MAX_COMMAND) { // mode 3 / headless
        packet[13+offset] = 0x02;
    }
    else if(PPM_data[AUX1] < PPM_MIN_COMMAND) { // mode 1
        packet[13+offset] = 0x00;
    }
    else { // mode 2
        packet[13+offset] = 0x01;
    }
    packet[14+offset] = 0x01;
    XN297_WritePayload(packet, packet_length);
}
