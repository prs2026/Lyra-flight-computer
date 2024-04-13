#if !defined(LCDLIB)
#define LCDLIB
// core libs
#include <Arduino.h>
#include <macros.h>

SX126x Lora;

class RADIO{

    uint16_t groundaddress = 0x1234; 
    uint16_t airaddress = 0xABCD; 

    uint8_t radiochannel = 68;

    const long frequency = 916e6;  // LoRa Frequency

    uint8_t sf = 7;                                                     // LoRa spreading factor: 7
    uint32_t bw = 125000;                                               // Bandwidth: 125 kHz
    uint8_t cr = 5;    

    public:

    int init();
    int recivepacket(telepacket packet);

};

int RADIO::init(){
    Serial.println("radio init start");
    SPI.setRX(MISO);
    SPI.setTX(MOSI);
    SPI.setSCK(SCLK);
    SPI.begin();
    Serial.println("radio spi start");
    Lora.setSPI(SPI);
    Lora.setPins(SXCS,SXRST,BUSY,-1,TXEN,RXEN);
    if(!Lora.begin()){
        Serial.println("lora init fail, cry");
    }
    Lora.setFrequency(frequency);
    Lora.setTxPower(22,SX126X_TX_POWER_SX1261);
    Lora.setLoRaModulation(sf, bw, cr);

    uint8_t headerType = SX126X_HEADER_EXPLICIT;                        // Explicit header mode
    uint16_t preambleLength = 12;                                       // Set preamble length to 12
    uint8_t payloadLength = 15;                                         // Initialize payloadLength to 15
    bool crcType = true;                                                // Set CRC enable
    Lora.setLoRaPacket(headerType, preambleLength, payloadLength, crcType);
    Lora.setSyncWord(0x3444);
    return 0;
}

int RADIO::recivepacket(telepacket packet){
    


    return 0;
}



#endif // LCDLIB