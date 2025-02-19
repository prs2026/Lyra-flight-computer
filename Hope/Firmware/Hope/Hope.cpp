#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi1
#define PIN_MISO 28
#define PIN_CS   25
#define PIN_SCK  26
#define PIN_MOSI 27

//Radio pins
#define PIN_DIO1 9
#define PIN_DIO2 1
#define PIN_DIO3 0
#define PIN_BUSY 10 // High when busy
#define PIN_TXCOEN 11 // Active high
#define PIN_RST 17 // active low


#define PIN_LED 8


// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart0
#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 12
#define UART_RX_PIN 13

//pulses the NRESET pin low for 10ms
//returns 0 
int resetsx1280(){
    gpio_put(PIN_RST,0);
    sleep_ms(10);
    gpio_put(PIN_RST,1);
    sleep_ms(10);
    printf("resetting sx1280\n");
    return 0;
}

int main()
{
    //Init standard io
    stdio_init_all();

    //wait so init messages can actually be seen on serial
    sleep_ms(2000);
    
    // SPI initialisation. This will use SPI at 12.5kHz.
    spi_init(SPI_PORT, 12500);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    //gpio inits
    gpio_init(PIN_CS);
    gpio_init(PIN_BUSY);
    gpio_init(PIN_DIO1);
    gpio_init(PIN_DIO2);
    gpio_init(PIN_DIO3);
    gpio_init(PIN_RST);
    gpio_init(PIN_LED);
    //Define pin directions
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_BUSY, GPIO_IN);
    gpio_set_dir(PIN_DIO1, GPIO_IN);
    gpio_set_dir(PIN_DIO2, GPIO_IN);
    gpio_set_dir(PIN_DIO3, GPIO_IN);
    gpio_set_dir(PIN_RST, GPIO_OUT);


    gpio_set_dir(PIN_LED, GPIO_OUT);
    
    // Chip select is active-low, so we'll initialise it to a driven-low state because theres nothing else there
    gpio_put(PIN_CS, 0);
    // TXCOEN is active high, so set high on startup
    gpio_put(PIN_CS, 1);
    // RST is active high, so set low on startup to set high later
    gpio_put(PIN_RST, 0);
    // Turn off LED until setup complete
    gpio_put(PIN_LED, 0);

    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    // reset the sx1280
    resetsx1280();

    while (gpio_get(PIN_BUSY))
    {
        printf("SX1280 BUSY\n");
        sleep_ms(200);
    }
    uint16_t address = 0x153;
    uint8_t addr_h = address >> 8;
    uint8_t addr_l = address & 0x00FF;

    uint8_t txdata[6] = {0x19,addr_h,addr_l,0xFF,0xFF,0xFF};
    uint8_t rxdata1[4] = {};
    gpio_put(PIN_CS,0);
    spi_write_blocking(spi1,txdata,4);
    spi_read_blocking(spi1,0xFF,rxdata1,4);
    gpio_put(PIN_CS,1);

    printf("recived data: %x %x %x %x\n",rxdata1[0],rxdata1[1],rxdata1[2],rxdata1[3]);
    printf("sent data: %x %x %x %x\n",txdata[0],txdata[1],txdata[2],txdata[3]);
    bool ledstatus = 0;
    while (true) {
        printf("Hello, world!\n");
        if (gpio_get(PIN_BUSY))
        {
            printf("SX1280 BUSY\n");
        }
        sleep_ms(1000);
        gpio_put(PIN_LED, ledstatus);
        ledstatus = !ledstatus;
    }
}



    // // Set up our UART
    // uart_init(UART_ID, BAUD_RATE);
    // // Set the TX and RX pins by using the function select on the GPIO
    // // Set datasheet for more information on function select
    // gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    