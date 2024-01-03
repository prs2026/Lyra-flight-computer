#if !defined(MACROS)
#define MACROS
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <ArduinoEigenDense.h>


#include <BMI088.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <E220.h>

#include "SPI.h"
#include <Wire.h>

#include <string.h>
#include "LittleFS.h"

//#include <ArduinoEigenDense.h>
//#include <ArduinoEigenDense.h>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;



#define LEDRED 12
#define LEDGREEN 13
#define LEDBLUE 14

#define BUZZERPIN 21

#define P1_EN 6
#define P2_EN 7
#define P3_EN 6
#define P4_EN 7


#define P1_CONT 28
#define P2_CONT 27
#define P3_CONT 28
#define P4_CONT 27

#define BATT_SENSE 2

#define SDA 22
#define SCL 23

#define OFF 0
#define RED 1
#define GREEN 2
#define BLUE 3

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

struct IMUdata{
    Vector3float accel;
    Vector3float gyro;
    float temp;
};

struct BAROdata{
    float pressure;
    float altitude;
    float temp;
    float verticalvel;
    float maxrecordedalt;
    float altitudeagl;
    float padalt;
};

struct variences{
    float alt;
    float vvel;
    float vaccel;
    float orientation;
};

struct position{
    Vector3float accel;
    float alt;
    float vvel;
};

union navpacket
{
    struct
    {
        int32_t errorflag;
        uint32_t uptime;
        IMUdata imudata;
        BAROdata barodata;
        Vector3float accelworld;
        Vector3float orientationeuler;
        Quatstruct orientationquat;
        Quatstruct orientationquatadj;
        variences uncertainty;
        position filtered;
        
    } r;
};

union mpstate{
    struct{
        int32_t errorflag;
        uint32_t uptime;
        uint32_t MET;
        uint32_t state;
        float batterystate;
    } r;
    uint8_t data8[sizeof(r)/sizeof(uint8_t)];
};


union logpacket{
    struct{
        uint8_t checksum1;
        mpstate MPstate;
        navpacket navsysstate;
        uint8_t checksum2;
    } r;
    uint8_t data[sizeof(r)/sizeof(uint8_t)];
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