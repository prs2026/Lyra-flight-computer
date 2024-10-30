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

int SENSECORE::startBaro(){
    int status = baro.begin(0x76);
    status ? coredata.errorflag = coredata.errorflag : coredata.errorflag = coredata.errorflag || 0b1;
    if (!status)
    {
        return status;
    }

    baro.setIIRFilter(IIR_FILTER_2);
    baro.setPresOversampling(OVERSAMPLING_X8);
    baro.setTempOversampling(OVERSAMPLING_SKIP);
    
    baro.startForcedConversion();
    baro.getMeasurements(coredata.baro.temp,coredata.baro.pressure,coredata.baro.altitude);
    return status;
}