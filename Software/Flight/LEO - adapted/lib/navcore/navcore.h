#if !defined(NAVCOREHEADER)
#define NAVCOREHEADER

#include <Lyrav2sensors.h>

//#define VERBOSETIMES

#if defined(LYRA_NAV_PROFILING)
enum NavProfileBucket : uint8_t
{
    NAV_PROF_LOOP_WORK,
    NAV_PROF_GETSENSORDATA,
    NAV_PROF_IMU,
    NAV_PROF_BARO,
    NAV_PROF_ADXL,
    NAV_PROF_MAG,
    NAV_PROF_SENSOR_POST,
    NAV_PROF_KFRUN,
    NAV_PROF_ACCEL_ADJUST,
    NAV_PROF_COUNT
};

static const char *const NAV_PROFILE_NAMES[NAV_PROF_COUNT] = {
    "loop_work",
    "getsensordata",
    "imu",
    "baro",
    "adxl",
    "mag",
    "sensor_post",
    "kfrun",
    "accel_adjust"
};

struct NavProfileBucketStats
{
    uint32_t count = 0;
    uint64_t totalUs = 0;
    uint32_t maxUs = 0;
};

struct NavProfileStats
{
    NavProfileBucketStats buckets[NAV_PROF_COUNT];

    void add(uint8_t bucket, uint32_t durationUs)
    {
        if (bucket >= NAV_PROF_COUNT)
        {
            return;
        }

        NavProfileBucketStats &stats = buckets[bucket];
        stats.count++;
        stats.totalUs += durationUs;
        if (durationUs > stats.maxUs)
        {
            stats.maxUs = durationUs;
        }
    }

    void reset()
    {
        for (uint8_t i = 0; i < NAV_PROF_COUNT; i++)
        {
            buckets[i] = {};
        }
    }
};

NavProfileStats NAV_PROFILE;

#define NAV_PROFILE_BEGIN(varname) uint32_t varname = micros()
#define NAV_PROFILE_END(bucket, varname) NAV_PROFILE.add((bucket), micros() - (varname))
#define NAV_PROFILE_ADD(bucket, durationUs) NAV_PROFILE.add((bucket), (durationUs))
#else
#define NAV_PROFILE_BEGIN(varname)
#define NAV_PROFILE_END(bucket, varname)
#define NAV_PROFILE_ADD(bucket, durationUs)
#endif

static Quatstruct normalizeQuatFast(Quatstruct q)
{
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm <= 0.0f)
    {
        return {1.0f, 0.0f, 0.0f, 0.0f};
    }

    float invNorm = 1.0f / norm;
    return {q.w*invNorm, q.x*invNorm, q.y*invNorm, q.z*invNorm};
}

static Quatstruct multiplyQuatFast(Quatstruct a, Quatstruct b)
{
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

static Vector3float rotateVectorFast(Quatstruct q, Vector3float v)
{
    q = normalizeQuatFast(q);

    float tx = 2.0f * (q.y*v.z - q.z*v.y);
    float ty = 2.0f * (q.z*v.x - q.x*v.z);
    float tz = 2.0f * (q.x*v.y - q.y*v.x);

    return {
        v.x + q.w*tx + (q.y*tz - q.z*ty),
        v.y + q.w*ty + (q.z*tx - q.x*tz),
        v.z + q.w*tz + (q.x*ty - q.y*tx)
    };
}

static Vector3float quatToEulerFast(Quatstruct q)
{
    q = normalizeQuatFast(q);

    float sinrCosp = 2.0f * (q.w*q.x + q.y*q.z);
    float cosrCosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    float roll = atan2f(sinrCosp, cosrCosp);

    float sinp = 2.0f * (q.w*q.y - q.z*q.x);
    sinp = fmaxf(-1.0f, fminf(1.0f, sinp));
    float pitch = asinf(sinp);

    float sinyCosp = 2.0f * (q.w*q.z + q.x*q.y);
    float cosyCosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
    float yaw = atan2f(sinyCosp, cosyCosp);

    return {roll, pitch, yaw};
}


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

KALMAN<Nstate,Nobs> K; // your Kalman filter
BLA::Matrix<Nobs> obs; // observation vector

IMU imu;
BARO baro;
ADXL adxl;
MAG magclass;

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

    

    

    float accumz = 0.0f;

    uint64_t hitlindex = 0;
    
    

    public:
        uint64_t hitltime = 0;
        uint8_t hitlteston = 0;
        navpacket _sysstate;

        uint8_t state;
        volatile uint8_t newdata = 0;

        int useaccel = 1;

        int ready = 0;

        int upsidedown = 0;

        int event = 0;

        int sendtoserial = 0;

        void KFinit();
        void startHitl();

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
        

        Quatstruct intergrategyros(float timestep);

        Quatstruct adjustwithaccel(float alpha);

        void upsidedowncheck();
        
        Vector3float getworldaccel(navpacket _state);

        void getpadoffset();
        void getcalibrationdata();
        void calcnewoffsets();
        void dumpoffsets();
};

void NAVCORE::KFinit(){
    _sysstate.r.filtered.alt = 0;
    _sysstate.r.filtered.vvel = 0;
    _sysstate.r.filtered.vertaccel = 0;
    
    prevsysstate = _sysstate;
    prevtime.kfupdate = micros();
    kfpredicttime = micros();
    _sysstate.r.orientationquat = adjustwithaccel(0);
}

void NAVCORE::startHitl(){
    hitltime = millis();
    hitlindex = 0;
    hitlteston = 1;
    baro.setforhitl();
}

NAVCORE::NAVCORE(){
    _sysstate.r.orientationquat = {1,0,0,0};
    _sysstate.r.errorflag = 0;
    _sysstate.r.filtered.maxalt = 0;
    Vector3d axis;
    axis << 1,0,0;
    Quaterniond tempquat(AngleAxisd(0.5*PI,axis));
    vectoradj = tempquat;

 //evolution matrix
    K.F = { 1.0,0.0,0.0,
            0.0,1.0,0.0,
            0.0,0.0,1.0};
    // measurement matrix
    K.H = { 1.0, 0.0,
            0.0, 0.0,
            0.0, 1.0};
    //measurement covariance matrix.
    K.R = {n1*n1,   0.0,
            0.0, n2*n2};
    // model covariance matrix. 
    K.Q = {m1*m1, 0.0,  0.0,
           0.0, m2*m2, 0.0,
           0.0, 0.0, m3*m3};

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
    Wire.begin();
    Wire.setClock(400000);
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
    barostatus = baro.init();
    barostatus ? _sysstate.r.errorflag || 0b1000 : _sysstate.r.errorflag;
    adxlstatus = adxl.init();
    barostatus ? _sysstate.r.errorflag || 0b10000 : _sysstate.r.errorflag;
    magclass.init();
    
    return 0;
}

void NAVCORE::getsensordata(bool readgps){
    uint64_t currentHitlIndex = hitlindex;

    if (hitlteston)
    {
        const uint64_t hitlCount = sizeof(hitldata)/sizeof(hitldata[0]);
        //Serial.println("hitltesting");
        while (hitlindex + 1 < hitlCount &&
               (hitldata[hitlindex][0]*1000)-(timetostart*1000) < millis() - hitltime)
        {
            //Serial.printf("%f,%d\n",hitldata[hitlindex][0]*1000,millis() - hitltime);
            hitlindex++;
        }

        if (hitlindex + 1 >= hitlCount)
        {
            hitlteston = 0;
            Serial.println("out of hitltesting");
        }

        currentHitlIndex = hitlindex;
    }
    

    #if !defined(VERBOSETIMES)
        const int imuOversampling = (state >= 1 && state <= 4) ? 1 : 5;
        const int hitlSampleIndex = (int)currentHitlIndex;
        NAV_PROFILE_BEGIN(imuProfileStartUs);
        imu.read(imuOversampling,hitlteston,hitlSampleIndex);
        NAV_PROFILE_END(NAV_PROF_IMU, imuProfileStartUs);

        NAV_PROFILE_BEGIN(baroProfileStartUs);
        baro.readsensor(hitlteston,hitlSampleIndex);
        NAV_PROFILE_END(NAV_PROF_BARO, baroProfileStartUs);

        NAV_PROFILE_BEGIN(adxlProfileStartUs);
        adxl.read(hitlteston,hitlSampleIndex);
        NAV_PROFILE_END(NAV_PROF_ADXL, adxlProfileStartUs);

        NAV_PROFILE_BEGIN(magProfileStartUs);
        static uint8_t magReadDivider = 0;
        if (!(state >= 1 && state <= 4) || magReadDivider == 0)
        {
            magclass.readsensor();
        }
        magReadDivider = (magReadDivider + 1) % 5;
        NAV_PROFILE_END(NAV_PROF_MAG, magProfileStartUs);
        //if(readgps){ gps.read();}
    #endif // VERBOSETIMES

    NAV_PROFILE_BEGIN(sensorPostProfileStartUs);
    
    if (useaccel == 1)
    {
        accumz += imu.data.accel.z;
        // if (millis() - invertedtime > 5000 )
        // {
        //     upsidedowncheck();
        //     invertedtime = millis();
        // }
    }

    
    if (upsidedown == 1)
    {
        imu.data.accel = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(imu.data.accel)) * upsidedownadj.inverse()) ));
        imu.data.gyro = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(imu.data.gyro)) * upsidedownadj.inverse()) ));
        adxl.data.accel = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(adxl.data.accel)) * upsidedownadj.inverse()) ));
        magclass.data.utesla = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(magclass.data.utesla)) * upsidedownadj.inverse()) ));
        magclass.data.gauss = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(magclass.data.gauss)) * upsidedownadj.inverse()) ));
    }
    
    if (imu.data.absaccel > 20 || adxl.data.absaccel > 40)
    {
        event = 1;
    }
    
    

    _sysstate.r.adxldata = adxl.data;
    _sysstate.r.imudata = imu.data;
    _sysstate.r.barodata = baro.data;
    _sysstate.r.magdata = magclass.data;
    NAV_PROFILE_END(NAV_PROF_SENSOR_POST, sensorPostProfileStartUs);
    
    return;
}

void NAVCORE::KFrun(){
    navpacket extrapolatedsysstate = _sysstate;

    float timestep = (micros() - kfpredicttime)/1e6f;

    if (_sysstate.r.filtered.vvel >= 200)
    {
        K.R = {n12*n12,   0.0,
                0.0, n2*n2};
    }
    else if (state >= 3)
    {
        K.R = {n1*n1,   0.0,
                0.0, n22*n22};
    }
    else
    {
        K.R = {n1*n1,   0.0,
                0.0, n2*n2};
    }

    obs(0) = _sysstate.r.barodata.altitudeagl;
    obs(1) = _sysstate.r.accelworld.z;

    float timestepSquared = timestep * timestep;
    K.F = {1.0,timestep,0.5f*timestepSquared,
           0.0, 1.0,timestep,
           0.0, 0.0, 1.0};

    K.update(obs);

    BLA::Matrix<3> newstate = K.getxcopy();

    extrapolatedsysstate.r.covariences.x = K.P(0,0);
    extrapolatedsysstate.r.covariences.y = K.P(1,1);
    extrapolatedsysstate.r.covariences.z = K.P(2,2);

    extrapolatedsysstate.r.filtered.alt = newstate(0); 
    extrapolatedsysstate.r.filtered.vvel = newstate(1); 
    extrapolatedsysstate.r.filtered.vertaccel = newstate(2);
    

    extrapolatedsysstate.r.orientationquat = intergrategyros(timestep);
    extrapolatedsysstate.r.orientationquatadj = normalizeQuatFast(extrapolatedsysstate.r.orientationquat);
    
    extrapolatedsysstate.r.accelworld = getworldaccel(extrapolatedsysstate);
    extrapolatedsysstate.r.orientationeuler = quatToEulerFast(extrapolatedsysstate.r.orientationquatadj);



    //Serial.printf(">extrap var: %f\n",extrapolatedsysstate.r.confidence.alt);

    kfpredicttime = micros();
    _sysstate = extrapolatedsysstate;
    _sysstate.r.filtered.alt > _sysstate.r.filtered.maxalt ? _sysstate.r.filtered.maxalt = _sysstate.r.filtered.alt : _sysstate.r.filtered.alt;
    // _data.altitudeagl > data.maxrecordedalt ? data.maxrecordedalt = _data.altitudeagl : data.maxrecordedalt = data.maxrecordedalt;

}                                                         

Quatstruct NAVCORE::intergrategyros(float timestep){
    Quatstruct orientationquat = _sysstate.r.orientationquat;
    Vector3float gyro = _sysstate.r.imudata.gyro;
    float gyroNorm = sqrtf(gyro.x*gyro.x + gyro.y*gyro.y + gyro.z*gyro.z);

    if (gyroNorm <= 0.000001f)
    {
        return normalizeQuatFast(orientationquat);
    }

    float halfAngle = 0.5f * timestep * gyroNorm;
    float scale = sinf(halfAngle) / gyroNorm;
    Quatstruct qdelta = {
        cosf(halfAngle),
        gyro.x * scale,
        gyro.y * scale,
        gyro.z * scale
    };

    orientationquat = multiplyQuatFast(orientationquat, qdelta);

    return normalizeQuatFast(orientationquat);
}


Quatstruct NAVCORE::adjustwithaccel(float alpha){
    Quaterniond orientationquat = quatstructtoeigen(_sysstate.r.orientationquat);
    Vector3d accel = vectorfloatto3(_sysstate.r.imudata.accel);

    Quaterniond accelquat;

    accelquat.x() = accel.x();
    accelquat.y() = accel.y();
    accelquat.z() = accel.z();

    accelquat = orientationquat * accelquat * orientationquat.inverse();

    accel.x() = accelquat.x();
    accel.y() = accelquat.y();
    accel.z() = accelquat.z();
    

    Vector3d accelnorm(accel.normalized());

    float phi = acos(accelnorm.y());

    Vector3d naxis(accelnorm.cross(Vector3d(0,1,0)));

    naxis = naxis.normalized();


    Quaterniond accelrotquat(AngleAxisd((1-alpha)*phi,naxis));

    orientationquat = accelrotquat * orientationquat;
    //Serial.println(">adjustingwithaccel: 1");

    return eigentoquatstruct(orientationquat.normalized());
}


Vector3float NAVCORE::getworldaccel(navpacket _state){
    Vector3float accelvec = _state.r.imudata.accel;
    float accelMag = sqrtf(accelvec.x*accelvec.x + accelvec.y*accelvec.y + accelvec.z*accelvec.z);
    
    if (accelMag > 20.0f*9.81f)
    {
        accelvec = _state.r.adxldata.accel;
    }

    accelvec = rotateVectorFast(_state.r.orientationquatadj, accelvec);

    Quatstruct sensorFrameAdjust = {0.70710678f, 0.70710678f, 0.0f, 0.0f};
    accelvec = rotateVectorFast(sensorFrameAdjust, accelvec);
    accelvec.z -= 9.801f;

    return accelvec;

};

void NAVCORE::getpadoffset(){
    baro.getpadoffset();
    _sysstate.r.filtered.maxalt = 0;
}

void NAVCORE::getcalibrationdata(){
    Serial.println("calibrating accels");
    IMUdata imucalidata = imu.readraw(50,1000);
    ADXLdata adxlcalidata = adxl.readraw(50,1000);
    double lowestval,highestval = 0;
    int index = 0;

    if (imucalidata.accel.x < lowestval)
    {
        lowestval = imucalidata.accel.x;
        index = -1;
    }
    if (imucalidata.accel.y < lowestval)
    {
        lowestval = imucalidata.accel.y;
        index = -2;
    }
    if (imucalidata.accel.z < lowestval)
    {
        lowestval = imucalidata.accel.z;
        index = -3;
    }

    if (imucalidata.accel.x > highestval)
    {
        lowestval = imucalidata.accel.x;
        index = 1;
    }
        if (imucalidata.accel.y > highestval)
    {
        lowestval = imucalidata.accel.y;
        index = 2;
    }
        if (imucalidata.accel.z > highestval)
    {
        lowestval = imucalidata.accel.z;
        index = 3;
    }
    
    switch (index)
    {
    case -1:
        imu.calibrationneg.x() = imucalidata.accel.x;
        adxl.calibrationneg.x() = adxlcalidata.accel.x;
        break;
    
    case -2:
        imu.calibrationneg.y() = imucalidata.accel.y;
        adxl.calibrationneg.y() = adxlcalidata.accel.y;
        break;
    
    case -3:
        imu.calibrationneg.z() = imucalidata.accel.z;
        adxl.calibrationneg.z() = adxlcalidata.accel.z;
        break;
    
    case 1:
        imu.calibrationpos.x() = imucalidata.accel.x;
        adxl.calibrationpos.x() = adxlcalidata.accel.x;
        break;
    
    case 2:
        imu.calibrationpos.y() = imucalidata.accel.y;
        adxl.calibrationpos.y() = adxlcalidata.accel.y;
        break;
    
    case 3:
        imu.calibrationpos.z() = imucalidata.accel.z;
        adxl.calibrationpos.z() = adxlcalidata.accel.z;
        break;

    
    
    default:
        break;
    }
    
    return;
}

void NAVCORE::calcnewoffsets(){
    Vector3d imubcal,adxlbcal;
    Matrix3d imuacal;

    imubcal << (imu.calibrationneg + imu.calibrationpos)/2;
    adxlbcal << (imu.calibrationneg + imu.calibrationpos)/2;

    imu.bcal = imubcal;
    adxl.bcal = adxlbcal;
    return;
}

void NAVCORE::dumpoffsets(){
    Serial.printf("BMI: %f,%f,%f \n ADXL: %f,%f,%f",imu.bcal.x(),imu.bcal.y(),imu.bcal.z(),adxl.bcal.x(),adxl.bcal.y(),adxl.bcal.z());
    return;
}

#endif // NAVCOREHEADER
