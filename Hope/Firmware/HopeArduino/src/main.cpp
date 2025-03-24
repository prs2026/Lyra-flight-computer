#include <Arduino.h>
#include <SPI.h>
#include <SX128XLT.h>   

#define MODETX
//#define MODERX


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

#define BAUD_RATE 9600

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 12
#define UART_RX_PIN 13


//settings

#define NSS PIN_CS
#define RFBUSY PIN_BUSY
#define NRESET PIN_DIO3
#define LED1 PIN_LED
#define DIO1 PIN_DIO1
#define RX_EN -1                                //pin for RX enable, used on some SX1280 devices, set to -1 if not used
#define TX_EN -1                                //pin for TX enable, used on some SX1280 devices, set to -1 if not used  
#define BUZZER -1                               //pin for BUZZER, set to -1 if not used 


#define LORA_DEVICE DEVICE_SX1280                //we need to define the device we are using  

//LoRa Modem Parameters
const uint32_t Frequency = 2445000000;           //frequency of transmissions
const int32_t Offset = 0;                        //offset frequency for calibration purposes
const uint8_t Bandwidth = LORA_BW_0400;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF7;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate

const int8_t TXpower = 10;                       //Power for transmissions in dBm

const uint16_t packet_delay = 1000;              //mS delay between packets





SX128XLT LT;                                                   //create a library class instance called LT

uint8_t TXPacketL;
uint32_t TXPacketCount, startmS, endmS;

uint8_t buff[] = "Hello World 1234567890";





void led_Flash(uint16_t flashes, uint16_t delaymS)
{
  uint16_t index;
  for (index = 1; index <= flashes; index++)
  {
    digitalWrite(LED1, HIGH);
    delay(delaymS);
    digitalWrite(LED1, LOW);
    delay(delaymS);
  }
}

void packet_is_Error()
{
  //if here there was an error transmitting packet
  uint16_t IRQStatus;
  IRQStatus = LT.readIrqStatus();                  //read the the interrupt register
  Serial.print(F(" SendError,"));
  Serial.print(F("Length,"));
  Serial.print(TXPacketL);                         //print transmitted packet length
  Serial.print(F(",IRQreg,"));
  Serial.print(IRQStatus, HEX);                    //print IRQ status
  LT.printIrqStatus();                             //prints the text of which IRQs set
}

void packet_is_OK()
{
  //if here packet has been sent OK
  uint16_t localCRC;

  Serial.print(F("  BytesSent,"));
  Serial.print(TXPacketL);                         //print transmitted packet length
  localCRC = LT.CRCCCITT(buff, TXPacketL, 0xFFFF);
  Serial.print(F("  CRC,"));
  Serial.print(localCRC, HEX);                     //print CRC of sent packet
  Serial.print(F("  TransmitTime,"));
  Serial.print(endmS - startmS);                   //print transmit time of packet
  Serial.print(F("mS"));
  Serial.print(F("  PacketsSent,"));
  Serial.print(TXPacketCount);                     //print total of packets sent OK
}







void setup( ) {
  delay(5000);
  Serial.begin(115200);

  pinMode(LED1, OUTPUT);                                   //setup pin as output for indicator LED
  led_Flash(2, 125);        
  pinMode(PIN_TXCOEN,OUTPUT);                               //two quick LED flashes to indicate program start
  digitalWrite(PIN_TXCOEN,HIGH);

  Serial.begin(9600);
  Serial.println();
  Serial.println(F("3_LoRa_Transmitter Starting"));

  SPI1.begin();

  //SPI beginTranscation is normally part of library routines, but if it is disabled in library
  //a single instance is needed here, so uncomment the program line below
  //SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //setup hardware pins used by device, then check if device is found

  if (LT.begin(NSS, NRESET, RFBUSY, DIO1, RX_EN, TX_EN, LORA_DEVICE))
  {
    Serial.println(F("LoRa Device found"));
    led_Flash(2, 125);                                   //two further quick LED flashes to indicate device found
    delay(1000);
  }
  else
  {
    Serial.println(F("No device responding"));
    while (1)
    {
      led_Flash(50, 50);                                 //long fast speed LED flash indicates device error
    }
  }

  //The function call list below shows the complete setup for the LoRa device using the information defined in the
  //Settings.h file.
  //The 'Setup LoRa device' list below can be replaced with a single function call;
  //LT.setupLoRa(Frequency, Offset, SpreadingFactor, Bandwidth, CodeRate);

  //***************************************************************************************************
  //Setup LoRa device
  //***************************************************************************************************
  LT.setMode(MODE_STDBY_RC);
  LT.setRegulatorMode(USE_LDO);
  LT.setPacketType(PACKET_TYPE_LORA);
  LT.setRfFrequency(Frequency, Offset);
  LT.setBufferBaseAddress(0, 0);
  LT.setModulationParams(SpreadingFactor, Bandwidth, CodeRate);
  LT.setPacketParams(12, LORA_PACKET_VARIABLE_LENGTH, 255, LORA_CRC_ON, LORA_IQ_NORMAL, 0, 0);
  LT.setDioIrqParams(IRQ_RADIO_ALL, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
  LT.setHighSensitivity();
  //***************************************************************************************************

  Serial.println();
  LT.printModemSettings();                               //reads and prints the configured LoRa settings, useful check
  Serial.println();
  LT.printOperatingSettings();                           //reads and prints the configured operating settings, useful check
  Serial.println();
  Serial.println();
  LT.printRegisters(0x900, 0x9FF);                       //print contents of device registers
  Serial.println();
  Serial.println();

  Serial.print(F("Transmitter ready"));
  Serial.println();

   
}

void loop() {

  #if defined(MODERX)


      
    #endif // MODERX

    #if defined(MODETX)

    Serial.print(TXpower);                                       //print the transmit power defined
    Serial.print(F("dBm "));
    Serial.print(F("Packet> "));
    Serial.flush();
  
    TXPacketL = sizeof(buff);                                    //set TXPacketL to length of array
    buff[TXPacketL - 1] = '*';                                   //replace null character at buffer end so its visible on reciver
  
    LT.printASCIIPacket(buff, TXPacketL);                        //print the buffer (the sent packet) as ASCII
  
    digitalWrite(LED1, HIGH);
    startmS =  millis();                                         //start transmit timer
    if (LT.transmit(buff, TXPacketL, 10000, TXpower, WAIT_TX))   //will return packet length sent if OK, otherwise 0 if transmit, timeout 10 seconds
    {
      endmS = millis();                                          //packet sent, note end time
      TXPacketCount++;
      packet_is_OK();
    }
    else
    {
      packet_is_Error();                                 //transmit packet returned 0, there was an error
    }
  
    digitalWrite(LED1, LOW);
    Serial.println();
    delay(packet_delay); 

    #endif // MODETX
}
