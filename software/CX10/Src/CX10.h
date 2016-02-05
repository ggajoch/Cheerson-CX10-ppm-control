#define GREEN_PACKET_LENGTH 15
#define BLUE_PACKET_LENGTH 19
#define RF_BIND_CHANNEL 0x02
#define BIND_COUNT 3000
#define NUM_RF_CHANNELS    4
#define BLUE_PACKET_PERIOD 6000
#define GREEN_PACKET_PERIOD 1500
static const uint8_t tx_rx_id[] = {0xCC,0xCC,0xCC,0xCC,0xCC};

#define CHANNELS 6
enum chan_order{  // TAER -> Spektrum chan order
    AILERON,
    THROTTLE,
    ELEVATOR,
    RUDDER,
    AUX1,  // mode (3 positions, highest position is headless on CX10-A)
    AUX2,  // flip control
};

#define PPM_MIN_COMMAND 1250
#define PPM_MID 1500
#define PPM_MAX_COMMAND 1750

enum{
    CX10_GREEN,
    CX10_BLUE, // also compatible with CX10-A, CX12
};

///////////////
// CX10 functions
void CX10_init();
void CX10_bind();
uint32_t process_CX10();
void CX10_Write_Packet(uint8_t init);
