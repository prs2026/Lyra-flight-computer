#include <Arduino.h>
#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         

#include "basiclib.h"
#include "sx1280lib.h"
#include <gpslib.h>

//#define MODEFLIGHT
#define MODESTATION

#if !defined(MODEFLIGHT)
#if !defined(MODESTATION)
#define MODEGROUND

#endif // MODE FLIGHT
#endif // MODE STATION
sx1280radio radio;
gpsinput gps;

const uint32_t checkinterval = 500;
uint32_t checktime;


void setup( ) {
  delay(3000);
  Serial.begin();
  Serial.println("init");
  pinMode( PIN_LED, OUTPUT);

  pinMode( PIN_TXCOEN, OUTPUT );
  digitalWrite( PIN_TXCOEN, HIGH );

  radio.initradio();

  Serial1.setRX(UART_RX_PIN);
  Serial1.setTX(UART_TX_PIN);

  Serial1.begin(9600);

  

  #if defined(MODEFLIGHT)
  radio.setuptorange(0x01);
  
  pinMode(UART_TX_PIN,OUTPUT);
  digitalWrite(UART_TX_PIN, HIGH);
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)

  radio.setuptorange(0x00);
  radio.settolisten();
  
  #endif // MODEGROUND

}

void loop() {
  #if defined(MODEFLIGHT)

  packet testtest;
  
  //radio.sendpacket(testtest);

  radio.pingrange();
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)

  if(millis() - checktime > checkinterval){

    checktime = millis();
    radio.checkforping();
  }

  gps.checkformessages();

  
  #endif // MODEGROUND

  #if defined(MODESTATION)
  
  radio.receivepacket();
  
  #endif // MODESTATION
  

  
}

