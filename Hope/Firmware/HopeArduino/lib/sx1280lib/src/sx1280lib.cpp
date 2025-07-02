#include "Arduino.h"
#include "SPI.h"
#include "sx1280lib.h"
#include <SX128XLT.h>                           //include the appropriate library  

#include "basiclib.h"




SX128XLT LT;                                    //create a library class instance called LT

#define NSS PIN_CS                                  //select pin on LoRa device
#define NRESET PIN_DIO3                                //reset pin on LoRa device 
#define RFBUSY PIN_BUSY                                //busy pin on LoRa device
#define LORA_DEVICE DEVICE_SX1280               //we need to define the device we are using
#define TXpower 10                              //LoRa transmit power in dBm
#define DIO1 PIN_DIO1


uint8_t TXPacketL;
uint32_t TXPacketCount;

uint8_t buff[] = "Hello World 1234567890";      //the message to send



#define RXBUFFER_SIZE 255                       //RX buffer size

uint32_t RXpacketCount;
uint32_t errors;

uint8_t RXBUFFER[RXBUFFER_SIZE];                //create the buffer that received packets are copied into

uint8_t RXPacketL;                              //stores length of packet received
int16_t PacketRSSI;                             //stores RSSI of received packet
int8_t  PacketSNR;                              //stores signal to noise ratio (SNR) of received packet



uint16_t rangeing_errors, rangeings_valid, rangeing_results;
uint16_t IrqStatus;
uint32_t endwaitmS, startrangingmS, range_result_sum, range_result_average;
float distance, distance_sum, distance_average;
bool ranging_error;
int32_t range_result;
int16_t RangingRSSI;

uint32_t response_sent;



void printElapsedTime()
{
  float seconds;
  seconds = millis() / 1000;
  Serial.print(seconds, 0);
  Serial.print(F("s"));
}




sx1280radio::sx1280radio(){
        return;
  }



int sx1280radio::initradio(){
    // bool setCS(pin_size_t pin); choosing to handle the CS pin in the library
  SPI1.setSCK( PIN_SCK ); // bool setSCK(pin_size_t pin);
  SPI1.setTX( PIN_MOSI );  // bool setTX(pin_size_t pin);
  SPI1.setRX( PIN_MISO );  // bool setRX(pin_size_t pin);
  SPI1.begin();

  Serial.println(F("3_LoRa_TransmitterIRQ Starting"));

  if (LT.begin(NSS, NRESET, RFBUSY, PIN_DIO1,LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    delay(1000);
  }
  else
  {
    Serial.println(F("No LoRa device responding"));
    while (1);
  }

  LT.setupLoRa(2445000000, 0, LORA_SF7, LORA_BW_0400, LORA_CR_4_5);      //configure frequency and LoRa settings

  Serial.print(F("Transmitter ready"));
  Serial.println();

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
    Serial.print(F("  "));
  Serial.print(TXpower);                                       //print the transmit power defined
  Serial.print(F(" dBm "));
  Serial.print(F(" Packet> \n"));
  
  Serial.flush();                                 //replace null character at buffer end so its visible on receiver

  if (LT.transmit((uint8_t *) &packetToSend, sizeof(packetToSend), 0, TXpower, WAIT_TX))  //will return packet length sent if OK, otherwise 0
  {
  }
  else
  {
    Serial.print(F("Send Error - IRQreg,"));
    Serial.print(LT.readIrqStatus(), HEX);
  }
    return 0;
}


packet sx1280radio::receivepacket(){

  packet recievedpacket;

  RXPacketL = LT.receive( (uint8_t *) &recievedpacket, sizeof(recievedpacket), 0, WAIT_RX); //wait for a packet to arrive with no timeout                     //something has happened, what I wonder ?

  PacketRSSI = LT.readPacketRSSI();
  PacketSNR = LT.readPacketSNR();

  if (RXPacketL == 0)
  {
    uint16_t IRQStatus;
    IRQStatus = LT.readIrqStatus();

    if (IRQStatus & IRQ_RX_TIMEOUT)
    {
      Serial.print(F("RXTimeout"));
    }
    else
    {
      errors++;
      Serial.print(F("PacketError"));
      Serial.print(F("IRQreg,"));
      Serial.print(IRQStatus, HEX);
    }
  }
  else
  {
    RXpacketCount++;
    Serial.print(RXpacketCount);
    Serial.print(F("  "));
    }

    Serial.println();
  
  return recievedpacket; 
}


float sx1280radio::pingrange(){
  uint8_t index;
  distance_sum = 0;
  range_result_sum = 0;
  rangeing_results = 0;  
  rangeing_errors = 0;
  rangeings_valid  = 0;                    //count of valid results in each loop

  for (index = 1; index <= rangeingcount; index++)
  {

    startrangingmS = millis();

    //Serial.println(F("Start Ranging"));

    LT.transmitRanging(RangingAddress, TXtimeoutmS, RangingTXPower, WAIT_TX);

    //Serial.println(F("Rangingafter1"));

    IrqStatus = LT.readIrqStatus();

    //Serial.println(F("Rangingafter3"));

    if (IrqStatus & IRQ_RANGING_MASTER_RESULT_VALID)
    {
      rangeing_results++;
      rangeings_valid++;
      //digitalWrite(LED1, HIGH);
      //Serial.print(F("Valid"));
      range_result = LT.getRangingResultRegValue(RANGING_RESULT_RAW);
      // Serial.print(F(",Register,"));
      // Serial.print(range_result);

      if (range_result > 800000)
      {
        range_result = 0;
      }
      range_result_sum = range_result_sum + range_result;

      float distanceskewfactor = 1.0;

      distance = LT.getRangingDistance(RANGING_RESULT_RAW, range_result, distanceskewfactor);
      distance_sum = distance_sum + distance;

      Serial.print(F("\n>Distance:"));
      Serial.print(distance, 1);
      //Serial.print(F(",RSSIReg,"));
      //Serial.print(LT.readRegister(REG_RANGING_RSSI));
      RangingRSSI = LT.getRangingRSSI();
      Serial.print(F("\n>RSSI:"));
      Serial.print(RangingRSSI);
      //Serial.print(F("dBm"));
      //digitalWrite(LED1, LOW);

      // Serial.print("\nE");
      // Serial.print(distance);
      // Serial.print(",");
      // Serial.print(RangingRSSI);
      // Serial.print("\n");
    }
    else
    {
      rangeing_errors++;
      distance = 0;
      range_result = 0;
      Serial.print(F("NotValid"));
      Serial.print(F(",Irq,"));
      Serial.print(IrqStatus, HEX);
    }
    delay(packet_delaymS);

    if (index == rangeingcount)
    {
      range_result_average = (range_result_sum / rangeing_results);

      if (rangeing_results == 0)
      {
        distance_average = 0;
      }
      else
      {
        distance_average = (distance_sum / rangeing_results);
      }

      Serial.print(F("\n>TotalValid:"));
      Serial.print(rangeings_valid);
      Serial.print(F("\n>TotalErrors:"));
      Serial.print(rangeing_errors);
      Serial.print(F("\n>AverageRAWResult:"));
      Serial.print(range_result_average);
      Serial.print(F("\n>AverageDistance:"));
      Serial.print(distance_average, 1);

#ifdef ENABLEDISPLAY
      display_screen1();
#endif

      //delay(2000);

    }
    Serial.println();
  }
  return distance_average;
}

void sx1280radio::checkforping(){
  // LT.receiveRanging(RangingAddress, 0, TXpower, NO_WAIT);

  // endwaitmS = millis() + rangingRXTimeoutmS;

  // while (!digitalRead(DIO1) && (millis() <= endwaitmS));          //wait for Ranging valid or timeout

  // if (millis() >= endwaitmS)
  // {
  //   //Serial.println("Error - Ranging Receive Timeout!!");
  //   //led_Flash(2, 100);                                             //single flash to indicate timeout
  // }
  // else
  // {
    IrqStatus = LT.readIrqStatus();
    //digitalWrite(LED1, HIGH);

    if (IrqStatus & IRQ_RANGING_SLAVE_RESPONSE_DONE)
    {
      response_sent++;
      //Serial.print(response_sent);
      //Serial.print(" Response sent");
    }
    else
    {
      //Serial.print("Slave error,");
      //Serial.print(",Irq,");
      //Serial.print(IrqStatus, HEX);
      //LT.printIrqStatus();
    }
    //digitalWrite(LED1, LOW);
    //Serial.println();
  //}
  return;
}

// 0x01 = masters 0x00 = slave
void sx1280radio::setuptorange(int role){
   LT.setupRanging(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate, RangingAddress, role);

  //LT.setRangingCalibration(Calibration);               //override automatic lookup of calibration value from library table

  Serial.println();
  //LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  //LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  //LT.printRegisters(0x900, 0x9FF);                       //print contents of device registers, normally 0x900 to 0x9FF
  Serial.println();
  Serial.println();
}
// 0x01 = masters 0x00 = slave
void sx1280radio::settolisten(){
  LT.receiveRanging(RangingAddress, 0xFFFF, TXpower, NO_WAIT);
  return;
}