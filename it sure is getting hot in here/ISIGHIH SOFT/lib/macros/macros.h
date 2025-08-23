#if !defined(MACROS)
#define MACROS
// core libs
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <hardware/sync.h>
#include <hardware/flash.h>
#include <ArduinoEigenDense.h>

// sensor libs
#include <BMI088.h>
#include <Adafruit_MAX31865.h>


//comms libs
#include "SPI.h"
#include <Wire.h>

// im not sure why this is there tbh
#include <string.h>

// log buf lib
#include <CircularBuffer.hpp>


//filter lib and dependecy
#include <Kalman.h>
#include <BasicLinearAlgebra.h>


using namespace BLA;

// define eigen things to use
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

// led pins
#define LEDPIN 8

// pin for buzzer
#define BUZZERPIN 0

// battery sense pin
#define BATT_SENSE 26
// i2c0 lines
#define SDA 0
#define SCL 1

#define SCLK 18 
#define MOSI 19
#define MISO 16
#define MAX1CS 25
#define MAX2CS 13 
// i2c0 is being used

// size of the buf to log to before launch is detected
#define LOGBUFSIZE (20*6)

// define the altitude to fire the main charge
#define MAINALT 400


// flash macros
#define FLASH_FILESYSTEM_SIZE 13631488 // 13MB
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - (FLASH_FILESYSTEM_SIZE-FLASH_SECTOR_SIZE))

//define basic data structs
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

struct Quatstruct{
    float w;
    float x;
    float y;
    float z;
};


// struct to hold all low G IMU data
struct IMUdata{
    Vector3float accel; //
    Vector3float gyro; //
    float absaccel; //
    float temp;//
};


// union to hold all data for the nav core that needs to be shared
union navpacket
{
    struct
    {
        uint16_t errorflag;//
        uint32_t uptime; //
        IMUdata imudata; //
        float temp1;
        float temp2;
    } r;
};

// union for all data relating to the MP core that needs to be shared
union mpstate{
    struct{
        uint16_t errorflag; //
        uint32_t uptime; //
        uint32_t MET; //
        uint8_t state; //
        uint16_t status; //
        int32_t missiontime; //
        float batterystate; //
    } r;
    uint8_t data8[sizeof(r)/sizeof(uint8_t)];
};

// define the struct to use to log to flash, makes adding checksums easier.
union logpacket{
    struct{
        uint8_t checksum1;
        mpstate MPstate;
        navpacket navsysstate;
        uint8_t checksum2;
    } r;
    uint8_t data[sizeof(r)/sizeof(uint8_t)];
};


#endif // MACROS