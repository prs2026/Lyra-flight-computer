#include <Arduino.h>
#include <SPI.h>
#include <sx1280OverSpi.h>
#include "basiclib.h"
#include "sx1280lib.h"

#define MODEFLIGHT

#if !defined(MODELFIGHT)
#define MODEGROUND

#endif // MODELFIGHT



//uint8_t antselPin = 28; // Setting variable for DLP-RFS1280 antenna select pin

sx1280radio radio;



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

    radio.initradio();
}

void loop() {
  //Serial.println("loop");
  digitalWrite( PIN_LED, HIGH );
  delay( 50 );
  digitalWrite( PIN_LED, LOW);

  #if defined(MODEFLIGHT)
  
  if (Serial.available())
  {
    packet testpacket;
    radio.sendpacket(testpacket);
    Serial.read();
  }
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)
  // /* Giving writeData an arbitrary size of 255 for payloadLength in sx1280Setup
  // Payload length does not matter for messages with headers */
  // for( i = 0; i < 255; i++){
  //   *( writeData + i ) = 0xFF;
  // }
  // writeData[ 254 ] = 0x00; // Ending the array with a NULL for a NULL terminated string

  // for( i = 0; i < 255; i++){
  //   *( readData + i ) = 0x00;
  // }

  radio.receivepacket();
  

  #endif // MODERSX
}


