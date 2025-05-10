#if !defined(SX1280LIB)
#define SX1280LIB

#include "Arduino.h"

class sx1280radio
{
private:
    uint8_t _CS_PIN;
    uint8_t _BUSY_PIN;
    uint8_t _RESET_PIN;
public:
    sx1280radio(uint8_t cssPin, 
                uint8_t busyPin, 
                uint8_t resetPin);
    ~sx1280radio();
        

    int initradio();

    void reset();

    int isbusy();

    int sendcommand(uint8_t opcode,uint8_t data[],uint8_t len);


};



#endif // SX1280LIB
