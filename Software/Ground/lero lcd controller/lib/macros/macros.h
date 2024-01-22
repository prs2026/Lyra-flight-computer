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

struct sentpacket{
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
};
struct packets
{
  Vector3int orientationeuler;
  Vector3int accel;
  Vector3int gyro;
  int16_t altitude;
  int16_t verticalvel;
  uint32_t uptime;
  uint8_t errorflagmp;
  uint8_t errorflagnav;
  uint8_t state;
  uint32_t dataage;
};

struct __attribute__((packed)) datapacket  
{
        uint8_t checksum;
        Vector3int orientationeuler;
        Vector3int accel;
        Vector3int gyro;
        int16_t altitude;
        int16_t verticalvel;
        uint32_t uptime;
        int8_t errorflagmp;
        int8_t errorflagnav;
        uint8_t state;
        uint32_t dataage;
        uint8_t checksum2;
        int16_t maxalt;
};

#endif // MACROS