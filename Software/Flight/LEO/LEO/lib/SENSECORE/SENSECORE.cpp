#include <SENSECORE.h>
#include <Arduino.h>

SENSECORE::SENSECORE()
{
}

SENSECORE::~SENSECORE()
{
}

int SENSECORE::I2Cinit(int printout){
    Wire.setSCL(SCL);
    Wire.setSDA(SDA);

    Wire.begin();

    scani23c(printout);
    return 0;
}