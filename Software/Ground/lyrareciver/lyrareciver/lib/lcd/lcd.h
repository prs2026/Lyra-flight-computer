#if !defined(LCDLIB3)
#define LCDLIB3
// core libs
#include <Arduino.h>
#include <macros.h>

#include <Arduino_GFX_Library.h>
Arduino_DataBus *bus = new Arduino_HWSPI(LCDDC /* DC */, LCDCS /* CS */, &SPI);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, LCDRST /* RST */);

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
    if(!gfx->begin()){
        Serial.println("display init fail ):");
    }

    gfx->fillScreen(BLACK);
    gfx->setCursor(10, 10);
    gfx->setTextColor(RED);
    gfx->println("Hello World!");
    Serial.println("out of lcd init");
    return 1;
}


#endif // LCDLIB