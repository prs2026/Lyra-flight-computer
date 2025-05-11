#if !defined(SX1280LIB)
#define SX1280LIB

#include "Arduino.h"
#include "basiclib.h"

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
};



#endif // SX1280LIB
