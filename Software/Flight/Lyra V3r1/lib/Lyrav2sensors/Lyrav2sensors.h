#if !defined(LYRAV2SENSORSLIB)
#define LYRAV2SENSORSLIB

// include the general lib and macros, which are included by the general lib
#include <generallib.h>


// sea level pressure, adjust to where you are
const float SEALEVELPRESSURE = 	1019.64;

/* accelunit object */
Bmi088Accel accelunit(Wire,0x18);
/* gyrounit object */
Bmi088Gyro gyrounit(Wire,0x68);

// adafruit bmp object
Adafruit_BMP3XX bmp;

// adafruit adxl object
Adafruit_ADXL375 adxl375((int32_t)12345,&Wire);

LLCC68 radio;


/*----------------------------------------------------------------------------------------*/
// imu class, holds all code for the bmi088 imu
class IMU{


// previous data, used mostly for error checking
IMUdata prevdata;

// low pass filter alphas
float gyroal = 0.3;
float accelal = 0.3;

public:
    IMU();
    IMUdata data;
    int init();
    void read(int oversampling = 5);
    IMUdata readraw(int oversampling = 5,int interval = 100);
    Vector3d calibrationpos;
    Vector3d calibrationneg;

    // calibration matrices
    Vector3d bcal;
    Matrix3d acal;


};
// constructer, assigns values to bcal and acal
IMU::IMU(){
        
        bcal <<-0,0,0;

        acal << 0,0,0
        ,0,0,0,
        0,0,0;
};

//inits the imu, returns 1 if the accel unit failed, and 2 if the gyro failed
int IMU::init(){
    int status;
    status = accelunit.begin();
    

    if (status < 0 )
    {
        Serial.print("accelunit init failure, error code: ");
        Serial.println(status);
        // MP.logtextentry("accelunit init failure, error code: ",status);
        return 1;
    }


    accelunit.setRange(accelunit.RANGE_24G);
    
    status = gyrounit.begin();
    if (status < 0 )
    {
        Serial.print("gyrounit init failure, error code: ");
        Serial.println(status);
        //MP.logtextentry("gyrounit init failure, error code: ",status);
        return 2;
    }
        Serial.println("IMU init success");
        //MP.logtextentry("IMU init sucess");
        return 0;
}

// reads the imu and applies calibration matrices
void IMU::read(int oversampling){
    IMUdata _data;
    Vector3d accel;
    Vector3d gyro;

    accel << 0,0,0;
    gyro << 0,0,0;
    
    for (int i = 0; i < oversampling; i++)
    {
        accelunit.readSensor();
        gyrounit.readSensor();

        accel.x() += accelunit.getAccelY_mss();
        accel.y() += accelunit.getAccelZ_mss();
        accel.z() += accelunit.getAccelX_mss();
        //Serial.printf("new accelmss z : %f \n",accel.z());

        gyro.x() += -gyrounit.getGyroX_rads();
        gyro.y() += -gyrounit.getGyroZ_rads();
        gyro.z() += gyrounit.getGyroY_rads();

        delayMicroseconds(1);
        gyro.x() < -73786 || gyro.x() > 73786 ? gyro.x() = data.gyro.x : gyro.x() = gyro.x();
        gyro.y() < -73786 || gyro.y() > 73786 ? gyro.y() = data.gyro.x : gyro.y() = gyro.y();
        gyro.z() < -73786 || gyro.z() > 73786 ? gyro.z() = data.gyro.x : gyro.z() = gyro.z();

        accel.x() < -7378 || accel.x() > 7378 ? accel.x() = data.accel.x : accel.x() = accel.x();
        accel.y() < -7378 || accel.y() > 7378 ? accel.y() = data.accel.x : accel.y() = accel.y();
        accel.z() < -7378 || accel.z() > 7378 ? accel.z() = data.accel.x : accel.z() = accel.z();
        
    }
    
    accel.x() /= oversampling;
    accel.y() /= oversampling;
    accel.z() /= oversampling;
    
    gyro.x() /= oversampling;
    gyro.y() /= oversampling;
    gyro.z() /= oversampling;

    // accel calibration
    accel = accel - bcal;

    accel = acal * accel;

    // low pass filter
    gyro.x() = gyroal*prevdata.gyro.x + (1-gyroal)*gyro.x();
    gyro.y() = gyroal*prevdata.gyro.y + (1-gyroal)*gyro.y();
    gyro.z() = gyroal*prevdata.gyro.z + (1-gyroal)*gyro.z();

    accel.x() = accelal*prevdata.accel.x + (1-accelal)*accel.x();
    accel.y() = accelal*prevdata.accel.y + (1-accelal)*accel.y();
    accel.z() = accelal*prevdata.accel.z + (1-accelal)*accel.z();

    //Serial.printf("new accelmss z : %f \n",accel.x());

    data.absaccel = accel.norm();
    _data.accel = vector3tofloat(accel);
    _data.gyro = vector3tofloat(gyro);

    _data.temp = accelunit.getTemperature_C();
    
    data = _data;
    prevdata = _data;
    
    return;
}

IMUdata IMU::readraw(int oversampling, int interval){
    IMUdata _data;
    Vector3d accel;
    Vector3d gyro;

    accel << 0,0,0;
    gyro << 0,0,0;
    
    for (int i = 0; i < oversampling; i++)
    {
        accelunit.readSensor();
        gyrounit.readSensor();

        accel.x() += accelunit.getAccelX_mss();
        accel.y() += accelunit.getAccelZ_mss();
        accel.z() += accelunit.getAccelY_mss();
        //Serial.printf("new accelmss z : %f \n",accel.z());

        gyro.x() += -gyrounit.getGyroX_rads();
        gyro.y() += -gyrounit.getGyroZ_rads();
        gyro.z() += gyrounit.getGyroY_rads();

        delayMicroseconds(interval);
        gyro.x() < -73786 || gyro.x() > 73786 ? gyro.x() = data.gyro.x : gyro.x() = gyro.x();
        gyro.y() < -73786 || gyro.y() > 73786 ? gyro.y() = data.gyro.x : gyro.y() = gyro.y();
        gyro.z() < -73786 || gyro.z() > 73786 ? gyro.z() = data.gyro.x : gyro.z() = gyro.z();

        accel.x() < -7378 || accel.x() > 7378 ? accel.x() = data.accel.x : accel.x() = accel.x();
        accel.y() < -7378 || accel.y() > 7378 ? accel.y() = data.accel.x : accel.y() = accel.y();
        accel.z() < -7378 || accel.z() > 7378 ? accel.z() = data.accel.x : accel.z() = accel.z();
        
    }

    accel.x() /= oversampling;
    accel.y() /= oversampling;
    accel.z() /= oversampling;
    
    gyro.x() /= oversampling;
    gyro.y() /= oversampling;
    gyro.z() /= oversampling;

    data.absaccel = accel.norm();
    _data.accel = vector3tofloat(accel);
    _data.gyro = vector3tofloat(gyro);

    _data.temp = accelunit.getTemperature_C();
    
    return _data;

}

/*--------------------------------------------------------------------------------------*/
class ADXL
{
private:


public:
    ADXLdata data;
    ADXL();
    int init();
    int read();
    int getnewoffsets();
    ADXLdata readraw(int oversampling = 5, int interval = 50);

    Vector3d calibrationpos;
    Vector3d calibrationneg;

    Vector3d offsets;
    Vector3d bcal;
    Matrix3d acal;
};

ADXL::ADXL()
{
    offsets << 0,0,0;

    // offsets << 0,0,0;

    // bcal << 0,0,0;
    // acal << 0,0,0;

    bcal << 0,0,0;

    acal << 0,	0,	0,
            0,	0,	0,
            0,	0,	0;
    return;
}

int ADXL::init()
{
    //Serial.println("starting adxl init");
    if (!adxl375.begin(0x53))
    {
        Serial.println("adxl init fail");
        return 1;
    }

    adxl375.setDataRate(ADXL3XX_DATARATE_400_HZ);
    //adxl375.printSensorDetails();
    adxl375.setTrimOffsets(0,0,0);
    Serial.println("adxl init sucess");
    
    return 0;
    
}

int ADXL::getnewoffsets(){
    int16_t x, y, z;
    x = adxl375.getX();
    y = adxl375.getY();
    z = adxl375.getZ();
    Serial.print("Raw X: "); Serial.print(x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(z); Serial.print("  ");Serial.println(" counts");

    // the trim offsets are in 'multiples' of 4, we want to round, so we add 2
    adxl375.setTrimOffsets(-(x+2)/4, -(y+2)/4, -(z-20+2)/4);  // Z should be '20' at 1g (49mg per bit)
    delay(100);
    int8_t x_offset, y_offset, z_offset;
    adxl375.getTrimOffsets(&x_offset, &y_offset, &z_offset);
    Serial.print("Current trim offsets: ");
    Serial.print(x_offset);  Serial.print(", ");
    Serial.print(y_offset);  Serial.print(", ");
    Serial.println(z_offset);
    return 0;
}

int ADXL::read()
{
    Vector3d _accel;

    sensors_event_t event;

    adxl375.getEvent(&event);

    _accel.x() = -event.acceleration.y;
    _accel.y() = -event.acceleration.z;
    _accel.z() = event.acceleration.x;

    _accel = _accel - offsets;

    _accel = _accel - bcal;

    //_accel = acal * _accel;

    data.absaccel = _accel.norm();

    data.accel = vector3tofloat(_accel);

    return 0;
}

ADXLdata ADXL::readraw(int oversampling, int interval){
    ADXLdata _data;
    Vector3d accel;

    accel << 0,0,0;
    
    for (int i = 0; i < oversampling; i++)
    {
        sensors_event_t event;
        adxl375.getEvent(&event);

        accel.x() = -event.acceleration.x;
        accel.y() = -event.acceleration.z;
        accel.z() = event.acceleration.y;

        delayMicroseconds(interval);

        accel.x() < -7378 || accel.x() > 7378 ? accel.x() = data.accel.x : accel.x() = accel.x();
        accel.y() < -7378 || accel.y() > 7378 ? accel.y() = data.accel.x : accel.y() = accel.y();
        accel.z() < -7378 || accel.z() > 7378 ? accel.z() = data.accel.x : accel.z() = accel.z();
        
    }

    accel.x() /= oversampling;
    accel.y() /= oversampling;
    accel.z() /= oversampling;

    _data.absaccel = accel.norm();

    _data.accel = vector3tofloat(accel);
    return _data;
}

/*--------------------------------------------------------------------------------------*/

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

BARO::BARO(){
    return;
}
    
int BARO::getpadoffset(int samplesize){
    double _padalt = 0;
    // for (int i = 0; i < samplesize; i++)
    // {
        _padalt = bmp.readAltitude(SEALEVELPRESSURE);
        delayMicroseconds(100);
    // }

    //_padalt /= samplesize;
    Serial.printf("new pad offset: %f\n",_padalt);
    //MP.logtextentry("BMP new pad offset: ",float(_padalt));
    data.padalt = _padalt;
    data.maxrecordedalt = 0;
    return 0;
}

int BARO::init(){
    if (!bmp.begin_I2C(0x76,&Wire))
    {
        Serial.println("BMP init failure");
        //MP.logtextentry("BMP init fail");
        return 1;
    }
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);
    getpadoffset();
    //Serial.println("BMP init success");
    //MP.logtextentry("BMP init fail");
    return 0;
}

void BARO::readsensor(){
    BAROdata _data;
    bmp.performReading();

    float timestep = (micros() - prevtime)/1e6;
    // float hpfgain = hpfcutoff / (2* M_PI * 1/timestep);

    _data.altitude = bmp.readAltitude(SEALEVELPRESSURE);//44330.0 * (1.0 - pow((bmp.pressure/100.0F) / SEALEVELPRESSURE, 0.1903));

    _data.altitude = lpfal*prevalt + (1-lpfal)*_data.altitude;
    // _data.altitude = _data.altitude - hpfstate;

    // hpfstate += hpfgain * _data.altitude;


    _data.altitudeagl = _data.altitude-data.padalt;
    _data.pressure = bmp.pressure;
    _data.temp = bmp.temperature;

    
    //Serial.printf(">timestep: %f \n",timestep);
    //prevverticalvel[address] = ((data.altitude - prevalt)/timestep);
    float deltaaltitude = _data.altitude - prevalt;
    //_data.verticalvel = lpfal*prevalt + (1-lpfal)*_data.altitude;
    //Serial.printf(">dalt: %f \n",deltaaltitude);

    prevverticalvel[address] = ((deltaaltitude)/timestep);//prevverticalvel[address];

    int j = 0;

    for (int i = 0; i < 5; i++)
    {
        _data.verticalvel += prevverticalvel[j];
        j++;
    }
    
    _data.verticalvel /= 5;

    _data.verticalvel = lpfal*data.verticalvel + (1-lpfal)*_data.verticalvel;

    address < 4 ? address++ : address = 0;

    _data.altitudeagl > data.maxrecordedalt ? data.maxrecordedalt = _data.altitudeagl : data.maxrecordedalt = data.maxrecordedalt;

    prevalt = _data.altitude;
    prevtime = micros();
    data = _data;
}
/*-------------------------------------------------------------------------------------*/


class SERIALPORT{
public:

    bool sendtoplot = true;
    SERIALPORT();
    int init();
    int senddata(mpstate state,navpacket navstate);

};

SERIALPORT::SERIALPORT(){}

int SERIALPORT::init(){
            Serial.begin(115200);

            uint32_t serialstarttime = millis();
            while (!Serial && millis() - serialstarttime < 3000);
            delay(100);

            if (Serial)
            {
                delay(100);
                Serial.println("\n\nMP Serial init");
                //MP.logtextentry("MP serial init");
                return 0;
            }
            else{
                return 1;
            }
    }

int SERIALPORT::senddata(mpstate state,navpacket navstate){
        if (sendtoplot)
        {

            Serial.printf(">MP uptime: %d \n",state.r.uptime);
            Serial.printf(">NAV uptime: %d \n",navstate.r.uptime);
            Serial.printf(">MISSIONTIME: %d \n",state.r.missiontime);
            Serial.printf(">MP errorflag %d \n" , state.r.errorflag);
            Serial.printf(">NAV errorflag %d \n", navstate.r.errorflag);
            Serial.printf(">accel x: %f \n",navstate.r.imudata.accel.x );
            Serial.printf(">accel y: %f \n",navstate.r.imudata.accel.y);
            Serial.printf(">accel z: %f \n",navstate.r.imudata.accel.z);
            Serial.printf(">accelabs: %f \n",navstate.r.imudata.absaccel);
            Serial.printf(">highaccel x: %f \n",navstate.r.adxldata.accel.x );
            Serial.printf(">highaccel y: %f \n",navstate.r.adxldata.accel.y);
            Serial.printf(">highaccel z: %f \n",navstate.r.adxldata.accel.z);
            Serial.printf(">highaccelabs: %f \n",navstate.r.adxldata.absaccel);
            Serial.printf(">accelworld x: %f \n",navstate.r.accelworld.x);
            Serial.printf(">accelworld y: %f \n",navstate.r.accelworld.y);
            Serial.printf(">accelworld z: %f \n"  ,navstate.r.accelworld.z);
            Serial.printf(">gyro x: %f \n",navstate.r.imudata.gyro.x*(180/M_PI) );
            Serial.printf(">gyro y: %f \n",navstate.r.imudata.gyro.y*(180/M_PI));
            Serial.printf(">gyro z: %f \n",navstate.r.imudata.gyro.z*(180/M_PI));
            Serial.printf(">altitude: %f \n" ,navstate.r.barodata.altitude);
            Serial.printf(">verticalvel: %f \n",navstate.r.barodata.verticalvel);
            Serial.printf(">filtered vvel: %f \n",navstate.r.filtered.vvel);
            Serial.printf(">orientation pitch: %f \n",navstate.r.orientationeuler.x*(180/M_PI));
            Serial.printf(">orientation yaw: %f \n",navstate.r.orientationeuler.y*(180/M_PI));
            Serial.printf(">orientation roll: %f \n",navstate.r.orientationeuler.z*(180/M_PI));

            Serial.printf(">maxrecorded alt: %f \n",navstate.r.barodata.maxrecordedalt);
            Serial.printf(">filtered alt: %f \n",navstate.r.filtered.alt);
            Serial.printf(">state : %d \n",state.r.state);
            Serial.printf(">altitudeagl : %f \n",navstate.r.barodata.altitudeagl);
            Serial.printf(">varience alt : %f \n",navstate.r.uncertainty.alt);
            Serial.printf(">varience vvel : %f \n",navstate.r.uncertainty.vvel);
            Serial.printf(">battery vol : %f \n",state.r.batterystate);
            Serial.printf(">baro temp : %f \n",navstate.r.barodata.temp);
            Serial.printf(">pyro cont : %d \n",state.r.pyroscont);
            Serial.printf(">pyros fired : %d \n",state.r.pyrosfired);
            Serial.printf(">pyro state: %d \n",state.r.pyrostate);
            return 0;
        }
        else
        {
            Serial.printf("%f,%f,%f \n",navstate.r.orientationeuler.x,navstate.r.orientationeuler.y,navstate.r.orientationeuler.z);
        }
        
        

        return 0;
    }
/*-------------------------------------------------------------------------------------------*/


class RADIO{

    uint16_t groundaddress = 0x1234; 
    uint16_t airaddress = 0xABCD; 

    uint8_t radiochannel = 68;

    const long frequency = 915E6;  // LoRa Frequency

    public:

    int init();
    int sendpacket(telepacket packet);

};

int RADIO::init(){
    SPI.setRX(MISO);
    SPI.setTX(MOSI);
    
    return 0;
}

int RADIO::sendpacket(telepacket packet){
    
    return 0;
}

#endif // LYRAV2SENSORSLIB