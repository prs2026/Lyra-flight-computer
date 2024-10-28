#if !defined(SENSECORE2)
#define SENSECORE2
#include <Arduino.h>
#include <BASIC.h>

// class for everything on core 1
class SENSECORE
{
private:
    // to see if maincore is ready
    
public:

    int status = 0;

    SENSECORE();
    ~SENSECORE();
    
    //init the i2c
    int I2Cinit(int printout);

};

#endif // SENSECORE