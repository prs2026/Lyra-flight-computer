#if !defined(LYRAV2SENSORSLIB)
#define LYRAV2SENSORSLIB

#include <generallib.h>



const float SEALEVELPRESSURE = 	1018.626;



class IMU{

Vector3d bcal;
Matrix3d acal;

IMUdata prevdata;

float gyroal = 0.3;
float accelal = 0.3;

public:
    IMU();
    IMUdata data;
    int init();
    void read(int oversampling = 32);

};


class BARO
{
float prevverticalvel[5];
float prevalt;

int address = 0;
uint64_t prevtime;

float lpfal = 0.7;
float lpfalv = 0.7;

public:
    BARO();
    BAROdata data;

    int getpadoffset(int samplesize = 1);
    int init();
    void readsensor();

};


class SERIALPORT{
public:

    bool sendtoplot = true;
    SERIALPORT();
    int init();
    int senddata(mpstate state,navpacket navstate);

};


class RADIO{

    uint16_t groundaddress = 0x1234; 
    uint16_t airaddress = 0xABCD; 

    uint8_t radiochannel = 68;

    public:

    int init();
    int sendpacket(telepacket packet);
    int setpower(int power);
};



#endif // LYRAV2SENSORSLIB