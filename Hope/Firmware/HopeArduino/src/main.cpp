#include <Arduino.h>
#include <SPI.h>
#include "sx1280lib.h"


#define MODETX
//#define MODERX


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

#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 12
#define UART_RX_PIN 13


// Arrays for passing data to and receiving data from sx1280 setup, rx, and tx functions
uint8_t writeData[ 255 ];
uint8_t readData[ 255 ];

//uint8_t antselPin = 28; // Setting variable for DLP-RFS1280 antenna select pin

uint32_t i = 0; // iterator

sx1280radio SX1280(PIN_CS,PIN_BUSY,PIN_DIO3);


void setup( ) {
  delay(3000);
  Serial.begin();
  Serial.println("init");
  pinMode( PIN_LED, OUTPUT);

  pinMode( PIN_TXCOEN, OUTPUT );
  digitalWrite( PIN_TXCOEN, HIGH );

  // bool setCS(pin_size_t pin); choosing to handle the CS pin in the library
  SPI1.setSCK( PIN_SCK ); // bool setSCK(pin_size_t pin);
  SPI1.setTX( PIN_MOSI );  // bool setTX(pin_size_t pin);
  SPI1.setRX( PIN_MISO );  // bool setRX(pin_size_t pin);
  SPI1.begin();

  

  
}

void loop() {
  Serial.println("loop");
  digitalWrite( PIN_LED, HIGH );
  delay( 50 );
  digitalWrite( PIN_LED, LOW);

}


