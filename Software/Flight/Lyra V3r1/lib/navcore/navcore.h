#if !defined(NAVCOREHEADER)
#define NAVCOREHEADER

#include <Lyrav2sensors.h>

//#define VERBOSETIMES

IMU imu;
BARO baro;
ADXL adxl;

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

    public:
    
    
        navpacket _sysstate;

        int useaccel = 1;

        int ready = 0;

        int upsidedown = 0;

        int event = 0;

        int sendtoserial = 0;

        void KFinit();

        NAVCORE();
        /*
        1 = no errors
        3 = failed handshake
        5 = i2c devices fail
        7 = accel init fail
        11 = gyro init fail
        13 = baro init fail
        17 = mag init fail
        19 = packet send fail
        negative = fatal error
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
        void getsensordata();

        void KFpredict();                                                         
        void KFupdate();
        

        Quatstruct intergrategyros(double timestep);

        Quatstruct adjustwithaccel(float alpha);

        void upsidedowncheck();
        
        Vector3float getworldaccel(navpacket _state);

        void getpadoffset();
        void getcalibrationdata();
        void calcnewoffsets();
        void dumpoffsets();

        

};

void NAVCORE::KFinit(){
    _sysstate.r.filtered.alt = _sysstate.r.barodata.altitudeagl;
    _sysstate.r.filtered.vvel = _sysstate.r.barodata.verticalvel;
    _sysstate.r.filtered.accel = _sysstate.r.accelworld;
    
    _sysstate.r.uncertainty.alt = 100;
    _sysstate.r.uncertainty.vvel = 100;
    _sysstate.r.uncertainty.vaccel = 100;
    prevsysstate = _sysstate;
    prevtime.kfupdate = micros();
    _sysstate.r.orientationquat = adjustwithaccel(0);
}

NAVCORE::NAVCORE(){
    _sysstate.r.orientationquat = {1,0,0,0};
    _sysstate.r.errorflag = 1;
    Vector3d axis;
    axis << 1,0,0;
    Quaterniond tempquat(AngleAxisd(0.5*PI,axis));
    vectoradj = tempquat;
};
/*
1 = no errors
3 = failed handshake
5 = i2c devices fail
7 = accel init fail
11 = gyro init fail
13 = baro init fail
17 = adxl init fail
19 = packet send fail
negative = fatal error
*/

int NAVCORE::initi2c(){
    Wire.setSCL(SCL);
    Wire.setSDA(SDA);
    Wire.setClock(10000);
    Wire.begin();
    scani2c(true) ? _sysstate.r.errorflag*= 5 : _sysstate.r.errorflag *= 1;
    return 0;
}

void NAVCORE::upsidedowncheck(){
    if (accumz > 0)
    {
        upsidedown = 0;
    }
    else
    {
        upsidedown = 1;
    }
    accumz = 0;
    
    return;
}

uint32_t NAVCORE::sensorinit(){
    int imustatus;
    int barostatus;
    int adxlstatus;
    imustatus = imu.init();;
    imustatus == 1 ? _sysstate.r.errorflag *= 7 : _sysstate.r.errorflag *= 1;
    imustatus == 2 ? _sysstate.r.errorflag *= 11 : _sysstate.r.errorflag *= 1;
    barostatus = baro.init();
    barostatus ? _sysstate.r.errorflag *= 13 : _sysstate.r.errorflag *= 1;
    adxlstatus = adxl.init();
    barostatus ? _sysstate.r.errorflag *= 17 : _sysstate.r.errorflag *= 1;

    if (adxlstatus != 0 || imustatus != 0 || barostatus != 0)
    {
        _sysstate.r.errorflag *= -1;
    }
    
    
    return 0;
}

void NAVCORE::getsensordata(){
    #if defined(VERBOSETIMES)
    if (sendtoserial)
    {
            
    uint32_t timestep = micros();
    imu.read();
    Serial.printf(">imureadtime: %f \n", float(micros()-timestep));
    timestep = micros();
    baro.readsensor();
    Serial.printf(">baroreadtime: %f \n", float(micros()-timestep));
    adxl.read();
    }
    else
    {
        imu.read();
        baro.readsensor();
        adxl.read();
    }

   #endif // VERBOSETIMES
    
    #if !defined(VERBOSETIMES)
        imu.read();
        baro.readsensor();
        adxl.read();
    #endif // VERBOSETIMES

    
    
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
        adxl.data.accel = vector3tofloat(quattovector(upsidedownadj *  (vectortoquat(vectorfloatto3(adxl.data.accel)) * upsidedownadj.inverse()) ));
    }
    
    if (imu.data.absaccel > 20 || adxl.data.absaccel > 40)
    {
        event = 1;
    }
    
    

    _sysstate.r.adxldata = adxl.data;
    _sysstate.r.imudata = imu.data;
    _sysstate.r.barodata = baro.data;
    return;
}

void NAVCORE::KFpredict(){
    navpacket extrapolatedsysstate = _sysstate;

    double timestep = (micros() - kfpredicttime)/1e6;

    Quaterniond adjquat = quatstructtoeigen(quatadj).normalized();


    extrapolatedsysstate.r.filtered.alt = _sysstate.r.filtered.alt + (timestep*_sysstate.r.filtered.vvel); // extrapolate with velocity dynamics
    extrapolatedsysstate.r.filtered.vvel = _sysstate.r.filtered.vvel + (timestep*_sysstate.r.accelworld.z); 


    extrapolatedsysstate.r.uncertainty.alt = _sysstate.r.uncertainty.alt + ((timestep*timestep)*_sysstate.r.uncertainty.vvel) + ALTVAR; // extrapolate variences with velocity dynamics
    extrapolatedsysstate.r.uncertainty.vvel = _sysstate.r.uncertainty.vvel + VVELVAR;


    extrapolatedsysstate.r.orientationquat = intergrategyros(timestep);
    extrapolatedsysstate.r.orientationquatadj = eigentoquatstruct(adjquat* (quatstructtoeigen(extrapolatedsysstate.r.orientationquat).normalized() * adjquat.inverse()));
    
    extrapolatedsysstate.r.accelworld = getworldaccel(extrapolatedsysstate);
    extrapolatedsysstate.r.orientationeuler = quat2euler(extrapolatedsysstate.r.orientationquatadj);

    //Serial.printf(">extrap var: %f\n",extrapolatedsysstate.r.confidence.alt);

    kfpredicttime = micros();
    _sysstate = extrapolatedsysstate;
}                                                         



void NAVCORE::KFupdate(){
    double timestep = (micros() - kfupdatetime)/1e6;

    variences kgain; // calc new kalman gain
    kgain.alt = _sysstate.r.uncertainty.alt/(_sysstate.r.uncertainty.alt+ALTNOISE);
    kgain.vvel = _sysstate.r.uncertainty.vvel/(_sysstate.r.uncertainty.vvel+VVELNOISE);
    //Serial.printf(">kalman gain: %f\n",kgain.alt);
    
    if (_sysstate.r.filtered.vvel < 240 || useaccel == 1)
    {
        _sysstate.r.filtered.alt = prevsysstate.r.filtered.alt + kgain.alt*(_sysstate.r.barodata.altitudeagl - prevsysstate.r.filtered.alt); // state update
        _sysstate.r.filtered.vvel = prevsysstate.r.filtered.vvel + kgain.vvel*(_sysstate.r.barodata.verticalvel - prevsysstate.r.filtered.vvel);
    
        _sysstate.r.uncertainty.alt = (1-kgain.alt)*prevsysstate.r.uncertainty.alt; // variences update
        _sysstate.r.uncertainty.vvel = (1-kgain.vvel)*prevsysstate.r.uncertainty.vvel;

    }


    _sysstate.r.filtered.alt > _sysstate.r.filtered.maxalt ? _sysstate.r.filtered.maxalt = _sysstate.r.filtered.alt : _sysstate.r.filtered.alt = _sysstate.r.filtered.alt;

    if (useaccel == 1)
    {
        _sysstate.r.orientationquat = adjustwithaccel(0.1);
    }

    prevsysstate = _sysstate;
    kfupdatetime = micros();
}

Quatstruct NAVCORE::intergrategyros(double timestep){
    Quaterniond orientationquat = quatstructtoeigen(_sysstate.r.orientationquat);
    Vector3d gyro = vectorfloatto3(_sysstate.r.imudata.gyro);
    
    Quaterniond qdelta(AngleAxisd(timestep*gyro.norm(), gyro.normalized()));

    orientationquat = orientationquat * qdelta;

    return eigentoquatstruct(orientationquat.normalized());
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
    Vector3d accelvec = vectorfloatto3(_state.r.imudata.accel);
    Quaterniond orientationquat3 = quatstructtoeigen(_state.r.orientationquatadj);//.inverse();

    Quaterniond accelquat2;

    accelquat2.x() = accelvec.x();
    accelquat2.y() = accelvec.y();
    accelquat2.z() = accelvec.z();

    accelquat2 = orientationquat3 * (accelquat2 * orientationquat3.inverse());

    accelquat2 = vectoradj * (accelquat2 * vectoradj.inverse());

    accelvec.x() = accelquat2.x();
    accelvec.y() = accelquat2.y();
    accelvec.z() = accelquat2.z();   
    

    
    Vector3d grav;
    grav << 0,0,9.801;
    Vector3d _accelworld = accelvec-grav;

    return vector3tofloat(_accelworld);

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