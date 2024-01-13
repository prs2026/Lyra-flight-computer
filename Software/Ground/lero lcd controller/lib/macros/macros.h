#if !defined(MACROS)
#define MACROS
#include <Arduino.h>
#include <KS0108_GLCD.h>


#define BUTTON1
#define BUTTON2
#define BUTTON3
#define BUTTON4
#define BUTTON5

#define LCDDI 2 // also cs
#define LCDRW 3 // also mosi
#define LCDE 4 // also sck
#define LCDRST 14

#define LCDCS1 16
#define LCDCS2 15

#define LCDDB0 10
#define LCDDB1 11
#define LCDDB2 12
#define LCDDB3 21
#define LCDDB4 20
#define LCDDB5 19
#define LCDDB6 18
#define LCDDB7 17

union packet
{
    struct macros
    {
        /* data */
    };
    
};


#endif // MACROS