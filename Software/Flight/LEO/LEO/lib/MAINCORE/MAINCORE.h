#if !defined(MAINCORE)
#include <Arduino.h>

// class for everything on core 1
class MAINCORE
{
private:
    // to see if maincore is ready
    
public:

    int status = 0;

    MAINCORE();
    ~MAINCORE();

};




#endif // OS