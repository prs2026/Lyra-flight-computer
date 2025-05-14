#include <Arduino.h>
#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         

#include "basiclib.h"
#include "sx1280lib.h"

#define MODEFLIGHT

#if !defined(MODEFLIGHT)
#define MODEGROUND

#endif // MODELFIGHT

sx1280radio radio;



void setup( ) {
  delay(3000);
  Serial.begin();
  Serial.println("init");
  pinMode( PIN_LED, OUTPUT);

  pinMode( PIN_TXCOEN, OUTPUT );
  digitalWrite( PIN_TXCOEN, HIGH );

  radio.initradio();

  

  #if defined(MODEFLIGHT)
  radio.setuptorange(0x01);
  
  pinMode(UART_TX_PIN,OUTPUT);
  digitalWrite(UART_TX_PIN, HIGH);
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)

  radio.setuptorange(0x00);
  
  #endif // MODEGROUND

}

void loop() {
  //Serial.println("loop");
  digitalWrite( PIN_LED, HIGH );
  delay( 500 );
  digitalWrite( PIN_LED, LOW);
 
  #if defined(MODEFLIGHT)

  packet testtest;
  
  //radio.sendpacket(testtest);

  radio.pingrange();
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)

  radio.listenforpings();
  
  //radio.receivepacket();

  #endif // MODERSX
}


