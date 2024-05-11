#if !defined(SDCARDLIB)
#define SDCARDLIB
// core libs
#include <Arduino.h>
#include <macros.h>

#define SPI_SPEED SD_SCK_MHZ(4)

class SDCARD
{
private:
    /* data */
public:
    SDCARD(/* args */);
    bool init();
};

SDCARD::SDCARD(/* args */)
{
}

bool SDCARD::init(){
    Serial.println("init start");
    if (!SD.begin(SDCS))
    {
        Serial.println("SD init fail");
        return 1;
    }

    File dataFile = SD.open("/datalog.txt", FILE_WRITE);
    if (!dataFile)
    {
        Serial.println("SD file init fail");
    }
    dataFile.print("testing");
    dataFile.close();
    return 0;
}


#endif // SDCARDLIB