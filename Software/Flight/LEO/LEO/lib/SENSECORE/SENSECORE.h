#if !defined(SENSECORE2)
#define SENSECORE2
#include <Arduino.h>
#include <BASIC.h>
#include <BMP388_DEV.h>


inline BMP388_DEV baro;
// class for everything on core 1
class SENSECORE
{
private:
    // to see if maincore is ready
    
    
public:

    SENSEDATA coredata;

    int status = 0;

    SENSECORE();
    ~SENSECORE();
    
    //init the i2c
    int I2Cinit(int printout);
    int startBaro();

};

#endif // SENSECORE