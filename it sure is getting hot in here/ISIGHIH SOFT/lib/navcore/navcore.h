#if !defined(NAVCOREHEADER)
#define NAVCOREHEADER

#include <Lyrav2sensors.h>

//#define VERBOSETIMES


// Dimensions of the matrices
#define Nstate 3 // length of the state vector
#define Nobs 2   // length of the measurement vector

// measurement std 
#define n1 0.1 // noise on baro when slow
#define n12 5 // noise on baro when fast (>200m/s as of 4/25)
#define n2 0.0186 // noise on accel
#define n22 5 // noise on accel when descending

// model std 
#define m1 0.01 //alt
#define m2 0.0005 //vel
#define m3 0.1 //accel

// KALMAN<Nstate,Nobs> K; // your Kalman filter
// BLA::Matrix<Nobs> obs; // observation vector

IMU imu;
MAX31865 tempsens;

class NAVCORE{

    Quatstruct quatadj = {1,0,0,0};
    Quaterniond vectoradj = {0.707,0,0.707,0};

    Quaterniond upsidedownadj = {0,1,0,0};


    const float ALTVAR = 0.01;
    const float VVELVAR = 0.12;
    const float VACCELVAR = 0.6;
    const float ORIENTVAR = 0.4;

    const float ALTNOISE = 0.1;
    const float VVELNOISE = 1.5;
    const float VACCELNOISE = 0.2;
    const float ORIENTNOISE = 0.4;


    
    navpacket prevsysstate;
    uint32_t kfupdatetime;
    uint32_t kfpredicttime;

    uint32_t invertedtime;

    

    

    double accumz = 0;

    uint64_t hitlindex = 0;
    
    

    public:
        uint64_t hitltime = 0;
        uint8_t hitlteston = 0;
        navpacket _sysstate;

        uint8_t state;
        uint8_t newdata;

        int useaccel = 1;

        int ready = 0;

        int upsidedown = 0;

        int event = 0;

        int sendtoserial = 0;

        void KFinit();

        NAVCORE();
        /*
        0 = no errors 
        1 = i2c devices fail
        10 = accel init fail
        100 = gyro init fail
        1000 = baro init fail
        10000 = adxl init fail
        100000 = mag init fail
        1000000 = gps init fail
        */
        struct timings{
            uint32_t sendpacket;
            uint32_t intergrateorientation;
            uint32_t kfupdate;
            uint32_t looptime;
            uint32_t getdata;
            uint32_t predictkf;
            uint32_t updatekf;
        };
        timings intervals[6] = {
            {50}, // ground idle
            {50}, // powered ascent
            {50}, // unpowered ascent
            {50}, // ballistic descent
            {50}, //ready to land
            {50} // landed
        }; 
        timings prevtime;


        int initi2c();

        uint32_t sensorinit();
        void getsensordata(bool readgps);

        void KFrun();                                                         
        

        Quatstruct intergrategyros(double timestep);

        Quatstruct adjustwithaccel(float alpha);

        void upsidedowncheck();
        
        Vector3float getworldaccel(navpacket _state);

        void getpadoffset();
        void getcalibrationdata();
        void calcnewoffsets();
        void dumpoffsets();
};

// void NAVCORE::KFinit(){
//     _sysstate.r.filtered.alt = 0;
//     _sysstate.r.filtered.vvel = 0;
//     _sysstate.r.filtered.vertaccel = 0;
    
//     prevsysstate = _sysstate;
//     prevtime.kfupdate = micros();
//     _sysstate.r.orientationquat = adjustwithaccel(0);
// }

NAVCORE::NAVCORE(){
    _sysstate.r.errorflag = 0;
    Vector3d axis;
    axis << 1,0,0;
    Quaterniond tempquat(AngleAxisd(0.5*PI,axis));
    vectoradj = tempquat;

    state = 0;
}
/*
0 = no errors 
1 = i2c devices fail
10 = accel init fail
100 = gyro init fail
1000 = baro init fail
10000 = adxl init fail
100000 = mag init fail
*/
int NAVCORE::initi2c(){
    Wire.setSCL(SCL);
    Wire.setSDA(SDA);
    Wire.setClock(10000);
    Wire.begin();
    scani2c(true) ? _sysstate.r.errorflag || 0b1 : _sysstate.r.errorflag;
    return 0;
}

void NAVCORE::upsidedowncheck(){
    if (accumz > 0)
    {
        upsidedown = 0;
        //Serial.println("rightsideup");
    }
    else
    {
        upsidedown = 1;
        //Serial.println("upsidedown");
    }
    accumz = 0;
    
    return;
}

uint32_t NAVCORE::sensorinit(){
    int imustatus;
    int barostatus;
    int adxlstatus;
    imustatus = imu.init();;
    imustatus == 1 ? _sysstate.r.errorflag || 0b10 : _sysstate.r.errorflag;
    imustatus == 2 ? _sysstate.r.errorflag || 0b100 : _sysstate.r.errorflag;
    tempsens.init();
    tempsens.read1(1);
    tempsens.init();
    tempsens.read2(1);
    return 0;
}

void NAVCORE::getsensordata(bool readgps){
    uint32_t hitlindex = 0;
    //Serial.printf("\n>NAVspot: %f \n", 1.5);
    if (hitlteston)
    {
        //Serial.println("hitltesting");
        while ((hitldata[hitlindex][0]*1000)-(timetostart*1000) < millis() - hitltime && hitlteston)
        {
            //Serial.printf("%f,%d\n",hitldata[hitlindex][0]*1000,millis() - hitltime);
            hitlindex++;
            if (hitlindex > (sizeof(hitldata)/sizeof(hitldata[0]))-1)
            {
                hitlteston = 0;
                Serial.println("out of hitltesting");
            }
        }
    }
    
    
    #if !defined(VERBOSETIMES)
        imu.read(5,hitlteston,hitlindex);
        //Serial.printf("\n>NAVspot: %f \n", 1.55);
        //if(readgps){ gps.read();}
        _sysstate.r.temp1 = tempsens.read1(0);
        _sysstate.r.temp2 = tempsens.read2(0);
        //Serial.printf("\n>NAVspot: %f \n", 1.6);
    #endif // VERBOSETIMES
    //Serial.printf("\n>NAVspot: %f \n", 1.75);
    if (useaccel == 1)
    {
        accumz += imu.data.accel.z;
        if (millis() - invertedtime > 5000 )
        {
            upsidedowncheck();
            invertedtime = millis();
        }
    }

    
    if (upsidedown == 1)
    {
        imu.data.accel = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(imu.data.accel)) * upsidedownadj.inverse()) ));
        imu.data.gyro = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(imu.data.gyro)) * upsidedownadj.inverse()) ));
    }
    
    if (imu.data.absaccel > 20)
    {
        event = 1;
    }
    
    

    _sysstate.r.imudata = imu.data;
    //Serial.printf("\n>NAVspot: %f \n", 1.875);
    return;
}

#endif // NAVCOREHEADER