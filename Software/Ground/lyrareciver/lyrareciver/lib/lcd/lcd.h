#if !defined(LCDLIB3)
#define LCDLIB3
// core libs
#include <Arduino.h>
#include <macros.h>

class LCDDISPLAY
{
private:
    /* data */
public:
    LCDDISPLAY(/* args */);
    bool init();
};

LCDDISPLAY::LCDDISPLAY(/* args */)
{
}

bool LCDDISPLAY::init(){
    Serial.println("init display");
    digitalWrite(LCDCS,LOW);
    int8_t returned = SPI.transfer(0xab);
    digitalWrite(LCDCS,HIGH);
    
    Serial.printf("recived: %d\n");
    Serial.println("out of lcd init");
    return 1;
}


#endif // LCDLIB