#if !defined(GPSLIB)
#define GPSLIB
#include <Arduino.h>
#include <basiclib.h>
//#include <TinyGPS++.h>


class gpsinput
{
private:

public:
    gpsinput();
    ~gpsinput();
        
    int checkformessages();
};

#endif // BASICLIB