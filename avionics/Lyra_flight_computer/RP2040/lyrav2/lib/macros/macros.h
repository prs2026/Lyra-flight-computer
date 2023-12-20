#if !defined(MACROS)
#define MACROS
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <ArduinoEigenDense.h>
#include <RF24.h>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

#define LEDRED 11
#define LEDGREEN 10
#define LEDBLUE 9

#define BUZZERPIN 5

#define P1_EN 6
#define P2_EN 7

#define SERVO1 12
#define SERVO2 13
#define SERVO3 14
#define SERVO4 15

#define P1_CONT 28
#define P2_CONT 27

#define BATT_SENSE 29


#define SDA 22
#define SCL 23

#define MAG_DRDY 8
#define MAG_INT 24
#define ACCEL_INT1 18
#define ACCEL_INT2 19
#define GYRO_INT3 20
#define GYRO_INT4 21
#define BARO_INT 25


#define SPI0_MISO 0
#define SPI0_SCLK 2
#define SPI0_MOSI 3

#define CS_SD 1
#define BRK_CS 4

#define UART0_TX 16
#define UART0_RX 17

#define OFF 0
#define RED 1
#define GREEN 2
#define BLUE 3

#define ALTVAR 0.5 
#define VVELVAR 0.8
#define VACCELVAR 0.6
#define ORIENTVAR 0.4

#define ALTNOISE 0.1
#define VVELNOISE 1.5
#define VACCELNOISE 0.2
#define ORIENTNOISE 0.4

#define NRF24 1;
#define E22 2;


byte radioaddress[][7] = {"flight","ground"};
uint8_t radioaddressebyte[2][4] = {{0xAB,0xCD},{0x12,0x34}};



RF24 radio(26,BRK_CS);

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
};

struct MAGdata{
    Vector3float gauss;
    Vector3float utesla;
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
        MAGdata magdata;
        Vector3float accelworld;
        //Vector3float pos;
        Vector3float orientationeuler;
        Quatstruct orientationquat;
        Quatstruct orientationquatadj;
        variences uncertainty;
        position filtered;
        //Vector3float vel;
        
    } r;
    uint32_t data[sizeof(r)/sizeof(uint32_t)];

};

union mpstate{
    struct{
        int32_t errorflag;
        uint32_t uptime;
        uint32_t MET;
        uint32_t state;
        float batterystate;
        //navpacket navsysstate;
    } r;
    uint32_t data[sizeof(r)/sizeof(uint32_t)];
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

telepacket statetopacket(mpstate state,navpacket navstate){
    telepacket packet;
    packet.r.checksum = 0x12;
    packet.r.checksum2 = 0x34;
    packet.r.accel.x = int16_t(navstate.r.imudata.accel.x*100);
    packet.r.accel.y = int16_t(navstate.r.imudata.accel.y*100);
    packet.r.accel.z = int16_t(navstate.r.imudata.accel.z*100);

    packet.r.gyro.x = int16_t(navstate.r.imudata.gyro.x*100);
    packet.r.gyro.y = int16_t(navstate.r.imudata.gyro.y*100);
    packet.r.gyro.z = int16_t(navstate.r.imudata.gyro.z*100);

    packet.r.orientationeuler.x = int16_t(navstate.r.orientationeuler.x*100);
    packet.r.orientationeuler.y = int16_t(navstate.r.orientationeuler.y*100);
    packet.r.orientationeuler.z = int16_t(navstate.r.orientationeuler.z*100);

    packet.r.uptime = state.r.uptime;
    packet.r.errorflagmp = state.r.errorflag;
    packet.r.errorflagnav = navstate.r.errorflag;
    packet.r.state = state.r.state;

    packet.r.altitude = int16_t(navstate.r.barodata.altitudeagl*10);
    packet.r.verticalvel = int16_t(navstate.r.barodata.verticalvel*100);
    return packet;
}

void printBin(byte aByte) {
  for (int8_t aBit = 7; aBit >= 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
}



#endif // MACROS