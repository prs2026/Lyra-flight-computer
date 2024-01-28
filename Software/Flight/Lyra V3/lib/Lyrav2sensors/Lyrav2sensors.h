#if !defined(LYRAV2SENSORSLIB)
#define LYRAV2SENSORSLIB

#include <generallib.h>



const float SEALEVELPRESSURE = 	1018.626;

/* accelunit object */
Bmi088Accel accelunit(Wire1,0x18);
/* gyrounit object */
Bmi088Gyro gyrounit(Wire1,0x68);

Adafruit_BMP3XX bmp;

Adafruit_ADXL375 adxl375((int32_t)12345,&Wire1);
// m0 = brkout2
// m1 = brkout7
// aux brkout 5
//tx = brkout 3
//rx = brkout 4

Stream &radioserial = (Stream &)Serial1;
                    //  m0         m1     aux
E220 ebyte(&radioserial,BRKOUT2,BRKOUT7,BRKOUT5);

/*----------------------------------------------------------------------------------------*/
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

IMU::IMU(){
        
        bcal << 0.01215,
                0.0558,
                -0.00645;

        acal << 1.0043134,	0.002845008234,	-0.0003288584787,
                0.0006194775994,	1.002656361,	-0.0003288584787,
                0.0006194775994,	0.002845008234,	1.006883084;
};

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

        accel.x() += accelunit.getAccelX_mss();
        accel.y() += accelunit.getAccelZ_mss();
        accel.z() += accelunit.getAccelY_mss();
        //Serial.printf("new accelmss z : %f \n",accel.z());

        gyro.x() += -gyrounit.getGyroX_rads();
        gyro.y() += -gyrounit.getGyroZ_rads();
        gyro.z() += gyrounit.getGyroY_rads();

        delayMicroseconds(5);
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

/*--------------------------------------------------------------------------------------*/
class ADXL
{
private:
    Vector3d offsets;
    Vector3d bcal;
    Matrix3d acal;

public:
    ADXLdata data;
    ADXL();
    int init();
    int read();
    int getnewoffsets();
};

ADXL::ADXL()
{
    offsets << -11.82755,
                8.92885,
                5.7293;

    // offsets << 0,0,0;

    // bcal << 0,0,0;
    // acal << 0,0,0;

    bcal << 0.0057,
            -2.7648,
            -0.07585;

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

    _accel.x() = -event.acceleration.x;
    _accel.y() = -event.acceleration.z;
    _accel.z() = event.acceleration.y;

    _accel = _accel - offsets;

    _accel = _accel - bcal;

    //_accel = acal * _accel;

    data.absaccel = _accel.norm();

    data.accel = vector3tofloat(_accel);

    return 0;
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
    if (!bmp.begin_I2C(0x76,&Wire1))
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

    public:

    int init();
    int sendpacket(telepacket packet);
    int setpower(int power);
};

int RADIO::init(){

        Serial1.end();
        Serial1.setRX(BRKOUT4);
        Serial1.setTX(BRKOUT3);
        Serial1.begin(9600);



        if (ebyte.init())
        {
            Serial.println("radio init sucess");
        }
        else
        {
            Serial.println("radio init fail");
            return 1;
        }
        
        

        ebyte.setAddress(0x1234,true);
        ebyte.setPower(Power_21,true);
        ebyte.setChannel(68,true);
        ebyte.setNetID(0,true);
        ebyte.setBaud(UDR_9600,true);
        ebyte.setSubPacketSize(SPS_64,true);
        ebyte.setAirDataRate(ADR_2400,true);
        ebyte.setEncryptionKey(0,true);
        ebyte.setLBT(true,true);
        ebyte.setFixedTransmission(true,true);
        ebyte.printBoardParameters();
        return 0;
    }

int RADIO::sendpacket(telepacket packet){
    ebyte.sendFixedData(groundaddress,radiochannel,packet.data,sizeof(packet.data),false);
    if (Serial1.available() && Serial1.read() == 0xFF)
    {
        ebyte.setRadioMode(MODE_NORMAL);
    }
    return 0;
}

int RADIO::setpower(int power){
    switch (power)
    {
    case 21:
        ebyte.setPower(Power_21,true);
        break;
    
    case 27:
        ebyte.setPower(Power_27,true);
        break;
    case 30:
        ebyte.setPower(Power_30,true);
        break;
    
    default:
        Serial.println("invailid power setting");
        break;
    }
    return 0;
}

#endif // LYRAV2SENSORSLIB