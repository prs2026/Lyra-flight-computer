#if !defined(BASICLIB)
#define BASICLIB


#define PIN_MISO 28
#define PIN_CS   25
#define PIN_SCK  26
#define PIN_MOSI 27

//Radio pins
#define PIN_DIO1 9
#define PIN_DIO2 1
#define PIN_DIO3 0 // jumpered to RST
#define PIN_BUSY 10 // High when busy
#define PIN_TXCOEN 11 // Active high
#define PIN_RST 17 // active low

#define PIN_LED 8

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 12
#define UART_RX_PIN 13

#define PIN_BATTSENSE A3

struct packet
{
    uint32_t uptime;
    float lat;
    float lon;
    float battvoltage;
    uint8_t command;
};

int parsecommand(uint8_t command);

float getbatteryvoltage();



#endif // BASICLIB