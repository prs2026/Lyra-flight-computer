#include <Arduino.h>
/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 08/02/20

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - The program listens for incoming packets using the LoRa settings in the 'Settings.h'
  file. The pins to access the lora device need to be defined in the 'Settings.h' file also.

  There is a printout of the valid packets received, the packet is assumed to be in ASCII printable text,
  if its not ASCII text characters from 0x20 to 0x7F, expect weird things to happen on the Serial Monitor.
  The LED will flash for each packet received and the buzzer will sound, if fitted.

  Sample serial monitor output;

  1109s  Hello World 1234567890*,CRC,DAAB,RSSI,-61dBm,SNR,9dB,Length,23,Packets,1026,Errors,0,IRQreg,50

  If there is a packet error it might look like this, which is showing a CRC error,

  1189s PacketError,RSSI,-111dBm,SNR,-12dB,Length,0,Packets,1126,Errors,1,IRQreg,70,IRQ_HEADER_VALID,IRQ_CRC_ERROR,IRQ_RX_DONE

  Serial monitor baud rate is set at 9600.
*******************************************************************************************************/

#define Station1address 16
#define Station1lat
#define station1lon

#define Station2address 17
#define Station2lat
#define station2lon

#define Station3address 18
#define Station3lat
#define station3lon

#define ThisStationAddress 16


#include <SPI.h>                                 //the lora device is SPI based so load the SPI library
#include <SX128XLT.h>                            //include the appropriate library   
#include <basiclib.h>

//*******  Setup hardware pin definitions here ! ***************

//These are the pin definitions for one of my own boards, the Easy Pro Mini,
//be sure to change the definitions to match your own setup.


#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         

#include "basiclib.h"
#include "sx1280lib.h"
#include <gpslib.h>

//#define MODEFLIGHT
//#define MODESTATION

#if !defined(MODEFLIGHT)
#if !defined(MODESTATION)
#define MODEGROUND

#endif // MODE FLIGHT
#endif // MODE STATION
sx1280radio radio;
gpsinput gps;

const uint32_t checkinterval = 500;
uint32_t checktime;

const uint32_t sendpacketinterval = 2000;
uint32_t sendpackettime;

uint8_t camstatus;


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
  Serial.print("flightuni");
  radio.setuptorange(0x01);
  
  pinMode(UART_TX_PIN,OUTPUT);
  digitalWrite(UART_TX_PIN, HIGH);
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)

  Serial.print("pingstation");

  radio.setuptorange(0x00);
  radio.settolisten(ThisStationAddress);
  
  #endif // MODEGROUND
  #if defined(MODESTATION)

  Serial.print("groundstation");
  
  #endif // MODEGROUND

}

void loop() {
  #if defined(MODEFLIGHT)

  packet testtest;

  testtest.r.lat = radio.pingrange(Station1address);

  testtest.r.uptime = millis();
  testtest.r.battvoltage = 4;
  testtest.r.command = camstatus;
  
  radio.sendpacket(testtest);

  delay(1000);
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)

  if(millis() - checktime > checkinterval){

    checktime = millis();
    radio.checkforping();
  }

  

  //gps.checkformessages();
  
  #endif // MODEGROUND

  #if defined(MODESTATION)
  
  packet newpacket = radio.receivepacket();

  //Serial.printf("\n>uptime: %d",newpacket.uptime);
  //Serial.printf("\n>battvolts: %f",newpacket.battvoltage);
  //Serial.printf("\n>status: %d",newpacket.command);
  #endif // MODESTATION
  

  
}

