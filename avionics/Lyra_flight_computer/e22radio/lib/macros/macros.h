#if !defined(MACROS)
#define MACROS
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

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
        variences confidence;
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



#endif // MACROS