#if !defined(SX1280LIB)
#define SX1280LIB

#include "Arduino.h"
#include "basiclib.h"
#include "SX128XLT.h"

class sx1280radio
{
private:
    uint8_t _CS_PIN;
    uint8_t _BUSY_PIN;
    uint8_t _RESET_PIN;

        // Arrays for passing data to and receiving data from sx1280 setup, rx, and tx functions
    uint8_t writeData[ 255 ];
    uint8_t readData[ 255 ];
public:
    sx1280radio();
    ~sx1280radio();
        

    int initradio();

    void reset();

    int isbusy();

    int sendpacket(packet packetToSend);

    packet receivepacket();

    void setuptorange(int role);

    float pingrange();
    void checkforping();
    void settolisten();

    void broadcastlocation();
};

//LoRa Modem Parameters
const uint32_t Frequency = 2445000000;           //frequency of transmissions in hz
const int32_t Offset = 0;                        //offset frequency in hz for calibration purposes
const uint8_t Bandwidth = LORA_BW_0800;          //LoRa bandwidth
const uint8_t SpreadingFactor = LORA_SF8;        //LoRa spreading factor
const uint8_t CodeRate = LORA_CR_4_5;            //LoRa coding rate
const uint16_t Calibration = 11350;              //Manual Ranging calibrarion value

const int8_t RangingTXPower = 10;                //Transmit power used
const uint32_t RangingAddress = 16;              //must match address in recever

const uint16_t  waittimemS = 10000;              //wait this long in mS for packet before assuming timeout
const uint16_t  TXtimeoutmS = 5000;              //ranging TX timeout in mS
const uint16_t  packet_delaymS = 0;              //forced extra delay in mS between ranging requests
const uint16_t  rangeingcount = 5;               //number of times ranging is cqarried out for each distance measurment
//float distance_adjustmentment = 1.0000;              //adjustment factor to calculated distance

const int8_t TXpower = 10;                       //Transmit power used

const uint16_t  rangingRXTimeoutmS = 100;     //ranging RX timeout in mS


#endif // SX1280LIB
