#include "Arduino.h"
#include "SPI.h"
#include "sx1280lib.h"
#include "sx1280OverSpi.h"
#include "basiclib.h"

sx1280OverSpi sx1280_1( PIN_CS,   // uint8_t cssPin
  PIN_BUSY,   // uint8_t busyPin
  PIN_DIO3,
  PIN_DIO1,
  PIN_DIO2); // uint8_t resetPin 


sx1280radio::sx1280radio(){
        return;}



int sx1280radio::initradio(){



    sx1280_1.begin();

    sx1280_1.sx1280Setup( 0x00,          /* uint8_t standbyMode              */
        0x01,          /* uint8_t packetType               */
        0xB8,          /* uint8_t rfFrequency2316          */
        0x9D,          /* uint8_t rfFrequency158           */
        0x89,          /* uint8_t rfFrequency70            */
        0x70,          /* uint8_t spreadingFactor          */
        0x0A,          /* uint8_t bandwidth                */
        0x01,          /* uint8_t codingRate               */
        0x0C,          /* uint8_t preambleLength           */
        0x01,          /* uint8_t headerType               */
        0x20,          /* uint8_t cyclicalRedundancyCheck  */
        0x40,          /* uint8_t chirpInvert              */
        writeData );   /* uint8_t outboundMessage[ ]       */
    return 1;
}

sx1280radio::~sx1280radio(){
    return;
}

void sx1280radio::reset(){
    digitalWrite(_RESET_PIN, 0);
    delay(10);
    digitalWrite(_RESET_PIN, 1);
    return;
}

int sx1280radio::isbusy(){
    return digitalRead(_BUSY_PIN);
}

int sx1280radio::sendpacket(packet packetToSend){
    uint32_t i = 0; // iterator

    for (int i = 0; i < 18; i++)
    {
        writeData[i] = packetToSend.data[i];
    }

    // writeData[0] = 0x01;
    // writeData[1] = 0x02;
    // writeData[2] = 0x00;
    // writeData[3] = 0x04;
    // writeData[4] = 0x05;
    // writeData[5] = 0x06;

    for( i = 18; i < 255; i++ ){
      writeData[ i ] = 0x00;
    }

    sx1280_1.sx1280Setup( 0x00,          /* uint8_t standbyMode              */
                          0x01,          /* uint8_t packetType               */
                          0xB8,          /* uint8_t rfFrequency2316          */
                          0x9D,          /* uint8_t rfFrequency158           */
                          0x89,          /* uint8_t rfFrequency70            */
                          0x70,          /* uint8_t spreadingFactor          */
                          0x0A,          /* uint8_t bandwidth                */
                          0x01,          /* uint8_t codingRate               */
                          0x0C,          /* uint8_t preambleLength           */
                          0x01,          /* uint8_t headerType               */
                          0x20,          /* uint8_t cyclicalRedundancyCheck  */
                          0x40,          /* uint8_t chirpInvert              */
                          writeData );   /* uint8_t outboundMessage[ ]       */

    sx1280_1.sx1280Tx( 0x1F,         /* uint8_t power                    */
                        0xE0,         /* uint8_t rampTime                 */
                        writeData,    /* uint8_t outboundMessage[ ]       */
                        0x40,         /* uint8_t txIrq158                 */
                        0x01,         /* uint8_t txIrq70                  */
                        0x02,         /* uint8_t txPeriodBase             */
                        0x01,         /* uint8_t txPeriodBaseCount158     */
                        0xF4,
                        18);       /* uint8_t txPeriodBaseCount70      */
    return 0;
}


packet sx1280radio::receivepacket(){
    packet recievedPacket;

    sx1280_1.sx1280Rx( 0x40,         /* uint8_t rxIrq158                 */
    0x7E,         /* uint8_t rxIrq70                  */
    0x02,         /* uint8_t rxPeriodBase             */
    0xFF,         /* uint8_t rxPeriodBaseCount158     */
    0xFF,         /* uint8_t rxPeriodBaseCount70      */
    recievedPacket.data );   /* uint8_t inboundMessage[ ]        */

  if (sx1280_1.rxflag == 1)
  {
    Serial.println("Packet Recieved");

    for (int i = 0; i < 20; i++)
    {
      Serial.printf("0x%x ",recievedPacket.data[i]);
    }
    Serial.println("");
    
    
    //sx1280_1.zeroingAnArray(readData,sizeof(readData));

    sx1280_1.rxflag = 0;
  }
  else
  {
    recievedPacket.r.checksum = 0xff;
  }
  
  return recievedPacket; 
}