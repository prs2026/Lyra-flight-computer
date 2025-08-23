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
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_LIS3MDL.h>

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
#define LEDPIN 7

// pin for buzzer
#define BUZZERPIN 0

// pyro enable pins
#define P1_EN 15
#define P2_EN 14
#define P3_EN 9
#define P4_EN 8

// pyro cont pins
#define P1_CONT 12
#define P2_CONT 14
#define P3_CONT 13
#define P4_CONT 11
// battery sense pin
#define BATT_SENSE 26
// i2c0 lines
#define SDA 0
#define SCL 1

//pins broken out to pads
#define BRKOUT1 25 //      SPI1 CSn  UART1 RX  I2C0 SCL
#define BRKOUT2 27 // ADC1 SPI1 TX   UART1 RTS  I2C1 SCL
#define BRKOUT3 28 // ADC2 SPI1 RX   UART0 TX   I2C0 SDA
#define BRKOUT4 29 // ADC3 SPI1 CSN  UART0 RX   I2C0 SCL

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

// struct to hold all high G IMU data
struct ADXLdata{
    Vector3float accel;//
    float absaccel;//
};

//struct to hold all barometer data
struct BAROdata{
    float pressure; //
    float altitude;//
    float temp;//
    float verticalvel;//
    float maxrecordedalt;//
    float altitudeagl;//
    float padalt;//
};

// struct to hold all magnetometer data
struct MAGdata{
    Vector3float gauss; //
    Vector3float utesla; //
};


//struct to hold various position related things so as to not bog down the main nav state struct
struct position{
    float vertaccel; //
    float alt;//
    float vvel;//
    float maxalt;//
};


// union to hold all data for the nav core that needs to be shared
union navpacket
{
    struct
    {
        uint16_t errorflag;//
        uint32_t uptime; //
        IMUdata imudata; //
        BAROdata barodata; //
        ADXLdata adxldata; //
        MAGdata magdata; //
        Vector3float accelworld; //
        Vector3float orientationeuler; //
        Quatstruct orientationquat; //
        Quatstruct orientationquatadj; //
        position filtered; //
        Vector3float covariences; //
    } r;
};

// union for all data relating to the MP core that needs to be shared
union mpstate{
    struct{
        uint16_t errorflag; //
        uint32_t uptime; //
        uint32_t MET; //
        uint8_t state; //
        uint8_t pyrosfired; //
        uint8_t pyroscont;//
        uint8_t pyrostate;//
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