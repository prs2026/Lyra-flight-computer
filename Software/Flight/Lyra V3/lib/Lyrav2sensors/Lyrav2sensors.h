#if !defined(Lyrav2sensors)
#define Lyrav2sensors

#include <generallib.h>
// #include <e22.h>


/* accelunit object */
Bmi088Accel accelunit(Wire1,0x18);
/* gyrounit object */
Bmi088Gyro gyrounit(Wire1,0x68);

Adafruit_BMP3XX bmp;

Stream &radioserial = (Stream &)Serial1;
                    //  m0         m1     aux
E220 ebyte(&radioserial,11,26,11);

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
    double padalt = 0;

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
};



#endif // Lyrav2sensors