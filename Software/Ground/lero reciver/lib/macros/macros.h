#if !defined(MACROS)
#define MACROS
#include <Arduino.h>
#include <E220.h>

#define LRC 15
#define BCLK 14
#define DIN 13

#define MISOSD 16
#define SCKSD 18
#define MOSISD 19
#define CS_SD 17

#define RADIOM0 4
#define RADIOM1 3
#define RADIOAUX 10

#define RADIORX 1
#define RADIOTX 0

#define NANOTX 8









struct Vector3float
{
    float x;
    float y;
    float z;
};

struct Vector3int{
    int16_t x;
    int16_t y;
    int16_t z;
};

union telepacket{
    struct 
    {
        uint8_t checksum;
        Vector3int orientationeuler;
        Vector3int accel;
        Vector3int gyro;
        int16_t altitude;
        int16_t verticalvel;
        uint32_t uptime;
        uint8_t errorflagmp;
        uint8_t errorflagnav;
        uint8_t state;
        uint8_t checksum2;
    } r;
    uint8_t data[sizeof(r)];

};

#endif // MACROS