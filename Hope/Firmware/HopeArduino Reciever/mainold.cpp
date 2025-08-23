#include <Arduino.h>
#include <SPI.h>
#include <sx1280OverSpi.h>
#include "basiclib.h"
#include "sx1280lib.h"

#define MODEFLIGHT

#if !defined(MODEFLIGHT)
#define MODEGROUND

#endif // MODELFIGHT



//uint8_t antselPin = 28; // Setting variable for DLP-RFS1280 antenna select pin

sx1280radio radio;

uint32_t lastpacketsent = 0;

void setup( ) {
  delay(3000);
  Serial.begin();
  Serial.println("init");
  pinMode( PIN_LED, OUTPUT);
  pinMode( PIN_DIO1, INPUT);
  pinMode( PIN_DIO2, INPUT);

  pinMode( PIN_TXCOEN, OUTPUT );
  digitalWrite( PIN_TXCOEN, HIGH );

  pinMode( PIN_BATTSENSE,INPUT);

  // bool setCS(pin_size_t pin); choosing to handle the CS pin in the library
  SPI1.setSCK( PIN_SCK ); // bool setSCK(pin_size_t pin);
  SPI1.setTX( PIN_MOSI );  // bool setTX(pin_size_t pin);
  SPI1.setRX( PIN_MISO );  // bool setRX(pin_size_t pin);
  SPI1.begin();

  radio.initradio();

  #if defined(MODEFLIGHT)
  pinMode(UART_TX_PIN,OUTPUT);
  digitalWrite(UART_TX_PIN, HIGH);
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)
  
  #endif // MODEGROUND

}

void loop() {
  //Serial.println("loop");
  digitalWrite( PIN_LED, HIGH );
  delay( 50 );
  digitalWrite( PIN_LED, LOW);
 
  #if defined(MODEFLIGHT)
  

  if (millis() - lastpacketsent > 2000)
  {
    Serial.print("Pinging with Ranging");
      float distance = radio.pingrange();

    Serial.printf("pinged range was: %f",distance);


    // packet testpacket;
    // testpacket.r.uptime = millis();
    // testpacket.r.command = 5;
    // testpacket.r.battvoltage = getbatteryvoltage();
    // testpacket.r.checksum = 0xAB;
    // testpacket.r.lat = 24.5;
    // testpacket.r.lat = 117.5;
    // radio.sendpacket(testpacket);
    lastpacketsent = millis();
  }

  // packet newpacket;
  // newpacket = radio.receivepacket();

  // if (newpacket.r.checksum == 0xAB)
  // {
  //   Serial.printf("uptime: %d, command %d , battery: %f \n",newpacket.r.uptime,newpacket.r.command,newpacket.r.battvoltage);
  //   Serial.printf(">uptime:%d\n",newpacket.r.uptime);
  //   Serial.printf(">command:%d\n",newpacket.r.command);
  //   Serial.printf(">battvoltage:%f\n",newpacket.r.battvoltage);
  //   parsecommand(newpacket.r.command);
  // }


  
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
  // packet newpacket;
  // newpacket = radio.receivepacket();

  // if (newpacket.r.checksum == 0xAB)
  // {
  //   Serial.printf("uptime: %d, command %d , battery: %f \n",newpacket.r.uptime,newpacket.r.command,newpacket.r.battvoltage);
  //   Serial.printf(">uptime:%d\n",newpacket.r.uptime);
  //   Serial.printf(">command:%x\n",newpacket.r.command);
  //   Serial.printf(">battvoltage:%f\n",newpacket.r.battvoltage);
  //   parsecommand(newpacket.r.command);
  // }

  // if (Serial.available())
  // {
  //   packet testpacket;
  //   testpacket.r.uptime = millis();
  //   testpacket.r.command = Serial.read();
  //   testpacket.r.battvoltage = getbatteryvoltage();
  //   testpacket.r.checksum = 0xAB;
  //   testpacket.r.lat = 24.5;
  //   testpacket.r.lat = 117.5;
  //   radio.sendpacket(testpacket);
  //   lastpacketsent = millis();
  // }

  if (millis() - lastpacketsent > 10000)
  {
    Serial.println("listening for pings");
    
  radio.listenforpings();
    lastpacketsent = millis();
  }
  
  

  #endif // MODERSX
}


