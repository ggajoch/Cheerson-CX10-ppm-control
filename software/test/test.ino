/*************************************************************
************* nRF24 Cheerson CX-10 Tx Code  ******************
*************     (Green & Blue boards)     ******************
**************************************************************
by goebish on RCgroups.com
Thanks to:
PhracturedBlue, victzh, hexfet, closedsink, midelic, Hasi ...
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License.
If not, see <http://www.gnu.org/licenses/>.
*/

#include <util/atomic.h>

///////////////////
// pinout

#define PPM_pin   2             // PPM in
//Spi Comm.pins with nRF24l01
#define CE_pin    3             // CE-D3
#define SCK_pin   4             // SCK-D4
#define MOSI_pin  5             // MOSI-D5
#define CS_pin    6             // CS-D6
#define MISO_pin  7             // MISO-D7

//---------------------------------
/*// spi outputs
#define  CS_on PORTD |= 0x40    // PORTD6
#define  CS_off PORTD &= 0xBF   // PORTD6
#define  CE_on PORTD |= 0x08    // PORTD3
#define  CE_off PORTD &= 0xF7   // PORTD3
//
#define  SCK_on PORTD |= 0x10   // PORTD4
#define  SCK_off PORTD &= 0xEF  // PORTD4
#define  MOSI_on PORTD |= 0x20  // PORTD5
#define  MOSI_off PORTD &= 0xDF // PORTD5
// spi input
#define  MISO_on (PIND & 0x80)  // PORTD7*/


#define  CS_on PORTH |= (1 << PH3)    // PORTH3
#define  CS_off PORTH &= (~(1 << PH3))   // PORTH3
#define  CE_on PORTE |= (1 << PE5)    // PORTE5
#define  CE_off PORTE &= (~(1 << PE5))   // PORTE5
//
#define  SCK_on PORTG |= (1 << PG5)   // PORTG5
#define  SCK_off PORTG &= (~(1 << PG5))  // PORTG5
#define  MOSI_on PORTE |= (1 << PE3)  // PORTE3
#define  MOSI_off PORTE &= (~(1 << PE3)) // PORTE3
// spi input
#define  MISO_on (PINH & (1 << PH4))  // PORTH4

//
#define _NOP() __asm__ __volatile__("nop")
#define BV(bit) (1 << bit)

#define GREEN_PACKET_LENGTH 15
#define BLUE_PACKET_LENGTH 19
#define RF_BIND_CHANNEL 0x02
#define BIND_COUNT 3000
#define NUM_RF_CHANNELS    4
#define BLUE_PACKET_PERIOD 6000
#define GREEN_PACKET_PERIOD 1500
static const uint8_t tx_rx_id[] = {0xCC,0xCC,0xCC,0xCC,0xCC};

#define PPM_MIN 1000
#define PPM_MIN_COMMAND 1250
#define PPM_MID 1500
#define PPM_MAX_COMMAND 1750
#define PPM_MAX 2000

// PPM stream settings
#define CHANNELS 6
enum chan_order{  // TAER -> Spektrum chan order
    AILERON,
    THROTTLE,
    ELEVATOR,
    RUDDER,
    AUX1,  // mode (3 positions, highest position is headless on CX10-A)
    AUX2,  // flip control
};

enum{
    CX10_GREEN, 
    CX10_BLUE, // also compatible with CX10-A, CX12
};

////////////////
// Variables
static uint8_t board_type = CX10_BLUE; // board type, change as required

static uint8_t txid[4]; // transmitter ID
static uint8_t freq[4]; // frequency hopping table
static uint8_t packet[BLUE_PACKET_LENGTH];
static uint8_t packet_length;
static uint32_t packet_period;
volatile uint16_t Servo_data[CHANNELS] = {0,};
static uint16_t ppm[CHANNELS] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,};
static uint8_t current_chan = 0;
static uint8_t ledPin = 13;

////////////////
// nRF24 Register map
enum {
    NRF24L01_00_CONFIG      = 0x00,
    NRF24L01_01_EN_AA       = 0x01,
    NRF24L01_02_EN_RXADDR   = 0x02,
    NRF24L01_03_SETUP_AW    = 0x03,
    NRF24L01_04_SETUP_RETR  = 0x04,
    NRF24L01_05_RF_CH       = 0x05,
    NRF24L01_06_RF_SETUP    = 0x06,
    NRF24L01_07_STATUS      = 0x07,
    NRF24L01_08_OBSERVE_TX  = 0x08,
    NRF24L01_09_CD          = 0x09,
    NRF24L01_0A_RX_ADDR_P0  = 0x0A,
    NRF24L01_0B_RX_ADDR_P1  = 0x0B,
    NRF24L01_0C_RX_ADDR_P2  = 0x0C,
    NRF24L01_0D_RX_ADDR_P3  = 0x0D,
    NRF24L01_0E_RX_ADDR_P4  = 0x0E,
    NRF24L01_0F_RX_ADDR_P5  = 0x0F,
    NRF24L01_10_TX_ADDR     = 0x10,
    NRF24L01_11_RX_PW_P0    = 0x11,
    NRF24L01_12_RX_PW_P1    = 0x12,
    NRF24L01_13_RX_PW_P2    = 0x13,
    NRF24L01_14_RX_PW_P3    = 0x14,
    NRF24L01_15_RX_PW_P4    = 0x15,
    NRF24L01_16_RX_PW_P5    = 0x16,
    NRF24L01_17_FIFO_STATUS = 0x17,
    NRF24L01_1C_DYNPD       = 0x1C,
    NRF24L01_1D_FEATURE     = 0x1D,
    //Instructions
    NRF24L01_61_RX_PAYLOAD  = 0x61,
    NRF24L01_A0_TX_PAYLOAD  = 0xA0,
    NRF24L01_E1_FLUSH_TX    = 0xE1,
    NRF24L01_E2_FLUSH_RX    = 0xE2,
    NRF24L01_E3_REUSE_TX_PL = 0xE3,
    NRF24L01_50_ACTIVATE    = 0x50,
    NRF24L01_60_R_RX_PL_WID = 0x60,
    NRF24L01_B0_TX_PYLD_NOACK = 0xB0,
    NRF24L01_FF_NOP         = 0xFF,
    NRF24L01_A8_W_ACK_PAYLOAD0 = 0xA8,
    NRF24L01_A8_W_ACK_PAYLOAD1 = 0xA9,
    NRF24L01_A8_W_ACK_PAYLOAD2 = 0xAA,
    NRF24L01_A8_W_ACK_PAYLOAD3 = 0xAB,
    NRF24L01_A8_W_ACK_PAYLOAD4 = 0xAC,
    NRF24L01_A8_W_ACK_PAYLOAD5 = 0xAD,
};

// Bit mnemonics
enum {
    NRF24L01_00_MASK_RX_DR  = 6,
    NRF24L01_00_MASK_TX_DS  = 5,
    NRF24L01_00_MASK_MAX_RT = 4,
    NRF24L01_00_EN_CRC      = 3,
    NRF24L01_00_CRCO        = 2,
    NRF24L01_00_PWR_UP      = 1,
    NRF24L01_00_PRIM_RX     = 0,

    NRF24L01_07_RX_DR       = 6,
    NRF24L01_07_TX_DS       = 5,
    NRF24L01_07_MAX_RT      = 4,

    NRF2401_1D_EN_DYN_ACK   = 0,
    NRF2401_1D_EN_ACK_PAY   = 1,
    NRF2401_1D_EN_DPL       = 2,
};

// Bitrates
enum {
    NRF24L01_BR_1M = 0,
    NRF24L01_BR_2M,
    NRF24L01_BR_250K,
    NRF24L01_BR_RSVD
};

enum TXRX_State {
    TXRX_OFF,
    TX_EN,
    RX_EN,
};

/////////////////
// setup
void setup() {
    Serial.begin(115200);
    randomSeed((analogRead(A0) & 0x1F) | (analogRead(A1) << 5));
    for(uint8_t i=0;i<4;i++) {
        txid[i] = random();
    }
    pinMode(ledPin, OUTPUT);
    //PPM input from transmitter port
    pinMode(PPM_pin, INPUT);
    //RF module pins
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
    digitalWrite(ledPin, LOW);//start LED off
    CS_on;//start CS high
    CE_on;//start CE high
    MOSI_on;//start MOSI high
    SCK_on;//start sck high
    delay(70);//wait 70ms
    CS_off;//start CS low
    CE_off;//start CE low
    MOSI_off;//start MOSI low
    SCK_off;//start sck low
    delay(100);
    CS_on;//start CS high
    delay(10);
    
    //**********************************************************************
    //PPM setup
    attachInterrupt(PPM_pin - 2, read_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz
    
    Serial.print("RST: "); Serial.println(NRF24L01_Reset());
    NRF24L01_Initialize();
    delay(150);

    
    CX10_init();
    delay(50);
    //Bind to Receiver
    CX10_bind();
}

///////////////
// main loop
void loop() {
    uint32_t timeout;
    // update ppm values out of ISR
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        ppm[ch] = Servo_data[ch];
        //Serial.print("Servo_data["); Serial.print(ch); Serial.print("] = "); Serial.println(Servo_data[ch]);
        //Serial.print(Servo_data[ch]); Serial.print(" ");
    }
    //Serial.println("\n");
    timeout = process_CX10();
    while(micros() < timeout)
    {   };
}

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
        for(uint8_t i=0; i<4; i++)
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
    delay(10);
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
    delay(150);
}

void CX10_bind()
{
    uint16_t counter=BIND_COUNT;
    bool bound=false;
    uint32_t timeout;
    while(!bound) {
        NRF24L01_SetTxRxMode(TX_EN);
        XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, RF_BIND_CHANNEL);
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
        NRF24L01_FlushTx();
        CX10_Write_Packet(0xAA); // send bind packet
        switch(board_type) {
            case CX10_GREEN:
                if(counter==0)
                    bound = true;
                delayMicroseconds(packet_period);
                break;
            case CX10_BLUE:    
                delay(1);
                CE_off;
                NRF24L01_SetTxRxMode(RX_EN);
                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();
                XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP) | BV(NRF24L01_00_PRIM_RX));
                CE_on;
                Serial.print("timeout: ");
                timeout = millis()+5;
                while(millis()<timeout) {
                    if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & BV(NRF24L01_07_RX_DR)) { // data received from aircraft
                        XN297_ReadPayload(packet, packet_length);
                        Serial.print("packet: ");
                        for(int i = 0; i < packet_length; ++i) {
                          Serial.print(packet[i]);
                          Serial.print(" ");
                        }
                        Serial.println("");
                        if( packet[9] == 0x01)
                            bound = true;
                        break;
                    }
                }                
                break;
        }
        digitalWrite(ledPin, counter-- & 0x10);
    }
    digitalWrite(ledPin, HIGH);
}

uint32_t process_CX10()
{
    uint32_t nextPacket = micros() + packet_period;
    XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, freq[current_chan++]);
    current_chan %= NUM_RF_CHANNELS;
    CX10_Write_Packet(0x55);
    return nextPacket;
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
    packet[5+offset] = lowByte(ppm[AILERON]);
    packet[6+offset]= highByte(ppm[AILERON]);
    Serial.print(ppm[THROTTLE]); Serial.print(" "); Serial.println(ppm[AILERON]);
    packet[7+offset]= lowByte(ppm[ELEVATOR]);
    packet[8+offset]= highByte(ppm[ELEVATOR]);
    packet[9+offset]= lowByte(ppm[THROTTLE]);
    packet[10+offset]= highByte(ppm[THROTTLE]);
    if(ppm[AUX2] > PPM_MID)
        packet[10+offset] |= 0x10; // flip flag
    packet[11+offset]= lowByte(ppm[RUDDER]);
    packet[12+offset]= highByte(ppm[RUDDER]);
    // rate / mode (use headless channel)
    if(ppm[AUX1] > PPM_MAX_COMMAND) { // mode 3 / headless
        packet[13+offset] = 0x02;
        //Serial.print(3);
    }
    else if(ppm[AUX1] < PPM_MIN_COMMAND) { // mode 1
        packet[13+offset] = 0x00;
        //Serial.print(2);
    }
    else { // mode 2
        packet[13+offset] = 0x01;
        //Serial.print(1);
    }
    packet[14+offset] = 0x01;
    XN297_WritePayload(packet, packet_length);
}

///////////////
// ppm input ISR
void read_ppm()
{
    #if F_CPU == 16000000
        #define PPM_SCALE 1L
    #elif F_CPU == 8000000
        #define PPM_SCALE 0L
    #else
        #error // 8 or 16MHz only !
    #endif
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    counterPPM = TCNT1;
    TCNT1 = 0;
    if(counterPPM < 510 << PPM_SCALE) {  //must be a pulse if less than 510us
        pulse = counterPPM;
    }
    else if(counterPPM > 1910 << PPM_SCALE) {  //sync pulses over 1910us
        chan = 0;
    }
    else{  //servo values between 510us and 2420us will end up here
        if(chan < CHANNELS) {
            Servo_data[chan]= constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
        }
        chan++;
    }
}

///////////////
// SPI

uint8_t _spi_write(uint8_t command)
{
    uint8_t result=0;
    uint8_t n=8;
    SCK_off;
    MOSI_off;
    while(n--) {
        if(command & 0x80)
            MOSI_on;
        else
            MOSI_off;
        if(MISO_on)
            result |= 0x01;
        SCK_on;
        _NOP();
        SCK_off;
        command = command << 1;
        result = result << 1;
    }
    MOSI_on;
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
    uint8_t result=0;
    uint8_t i;
    MOSI_off;
    _NOP();
    for(i=0;i<8;i++) {
        if(MISO_on) // if MISO is HIGH
            result = (result<<1)|0x01;
        else
            result = result<<1;
        SCK_on;
        _NOP();
        SCK_off;
        _NOP();
    }
    return result;
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

///////////////
// nRF24

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

static uint8_t rf_setup;

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
    for(uint8_t i=0;i<len;i++)
        _spi_write(data[i]);
    CS_on;
    delayMicroseconds(5);
}

void NRF24L01_Initialize()
{
    rf_setup = 0x0F;
}

uint8_t NRF24L01_FlushTx()
{
    return Strobe(FLUSH_TX);
}

uint8_t NRF24L01_FlushRx()
{
    return Strobe(FLUSH_RX);
}

static uint8_t Strobe(uint8_t state)
{
    uint8_t result;
    CS_off;
    result = _spi_write(state);
    CS_on;
    return result;
}

uint8_t NRF24L01_WritePayload(uint8_t *data, uint8_t length)
{
    CE_off;
    CS_off;
    _spi_write(W_TX_PAYLOAD);
    for(uint8_t i=0; i<length; i++)
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
    Serial.print("Status 1 "); Serial.println(status1);
    uint8_t status2 = NRF24L01_ReadReg(0x07);
    Serial.print("Status 2 "); Serial.println(status2);
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

///////////////
// XN297 emulation layer

static uint8_t xn297_addr_len;
static uint8_t xn297_tx_addr[5];
static uint8_t xn297_rx_addr[5];
static uint8_t xn297_crc = 0;

static const uint8_t xn297_scramble[] = {
    0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
    0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
    0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
    0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
    0x8e, 0xc5, 0x2f};

static const uint16_t xn297_crc_xorout[] = {
    0x0000, 0x3448, 0x9BA7, 0x8BBB, 0x85E1, 0x3E8C,
    0x451E, 0x18E6, 0x6B24, 0xE7AB, 0x3828, 0x814B,
    0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
    0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401,
    0x2138, 0x129F, 0xB3A0, 0x2988};

static uint8_t bit_reverse(uint8_t b_in)
{
    uint8_t b_out = 0;
    for (int i = 0; i < 8; ++i) {
        b_out = (b_out << 1) | (b_in & 1);
        b_in >>= 1;
    }
    return b_out;
}

static const uint16_t polynomial = 0x1021;
static const uint16_t initial    = 0xb5d2;
static uint16_t crc16_update(uint16_t crc, unsigned char a)
{
    crc ^= a << 8;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ polynomial;
            } else {
            crc = crc << 1;
        }
    }
    return crc;
}

void XN297_SetTXAddr(const uint8_t* addr, int len)
{
    if (len > 5) len = 5;
    if (len < 3) len = 3;
    uint8_t buf[] = { 0x55, 0x0F, 0x71, 0x0C, 0x00 }; // bytes for XN297 preamble 0xC710F55 (28 bit)
    xn297_addr_len = len;
    if (xn297_addr_len < 4) {
        for (int i = 0; i < 4; ++i) {
            buf[i] = buf[i+1];
        }
    }
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, len-2);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, buf, 5);
    // Receive address is complicated. We need to use scrambled actual address as a receive address
    // but the TX code now assumes fixed 4-byte transmit address for preamble. We need to adjust it
    // first. Also, if the scrambled address begins with 1 nRF24 will look for preamble byte 0xAA
    // instead of 0x55 to ensure enough 0-1 transitions to tune the receiver. Still need to experiment
    // with receiving signals.
    memcpy(xn297_tx_addr, addr, len);
}

void XN297_SetRXAddr(const uint8_t* addr, int len)
{
    if (len > 5) len = 5;
    if (len < 3) len = 3;
    uint8_t buf[] = { 0, 0, 0, 0, 0 };
    memcpy(buf, addr, len);
    memcpy(xn297_rx_addr, addr, len);
    for (int i = 0; i < xn297_addr_len; ++i) {
        buf[i] = xn297_rx_addr[i] ^ xn297_scramble[xn297_addr_len-i-1];
    }
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, len-2);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, buf, 5);
}

void XN297_Configure(uint8_t flags)
{
    xn297_crc = !!(flags & BV(NRF24L01_00_EN_CRC));
    flags &= ~(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, flags);
}

uint8_t XN297_WritePayload(uint8_t* msg, int len)
{
    uint8_t buf[32];
    uint8_t res;
    int last = 0;
    if (xn297_addr_len < 4) {
        // If address length (which is defined by receive address length)
        // is less than 4 the TX address can't fit the preamble, so the last
        // byte goes here
        buf[last++] = 0x55;
    }
    for (int i = 0; i < xn297_addr_len; ++i) {
        buf[last++] = xn297_tx_addr[xn297_addr_len-i-1] ^ xn297_scramble[i];
    }

    for (int i = 0; i < len; ++i) {
        // bit-reverse bytes in packet
        uint8_t b_out = bit_reverse(msg[i]);
        buf[last++] = b_out ^ xn297_scramble[xn297_addr_len+i];
    }
    if (xn297_crc) {
        int offset = xn297_addr_len < 4 ? 1 : 0;
        uint16_t crc = initial;
        for (int i = offset; i < last; ++i) {
            crc = crc16_update(crc, buf[i]);
        }
        crc ^= xn297_crc_xorout[xn297_addr_len - 3 + len];
        buf[last++] = crc >> 8;
        buf[last++] = crc & 0xff;
    }
    res = NRF24L01_WritePayload(buf, last);
    return res;
}

uint8_t XN297_ReadPayload(uint8_t* msg, int len)
{
    uint8_t res = NRF24L01_ReadPayload(msg, len);
    for(uint8_t i=0; i<len; i++)
        msg[i] = bit_reverse(msg[i]) ^ bit_reverse(xn297_scramble[i+xn297_addr_len]);
    return res;
}
