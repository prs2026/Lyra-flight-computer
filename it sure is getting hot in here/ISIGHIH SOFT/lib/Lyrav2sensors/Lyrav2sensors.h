#if !defined(LYRAV2SENSORSLIB)
#define LYRAV2SENSORSLIB

// include the general lib and macros, which are included by the general lib
#include <generallib.h>
#include <hitltestdata.h>


// sea level pressure, adjust to where you are
const float SEALEVELPRESSURE = 	1019.64;

/* accelunit object */
Bmi088Accel accelunit(Wire,0x18);
/* gyrounit object */
Bmi088Gyro gyrounit(Wire,0x68);

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX1CS);
Adafruit_MAX31865 thermo2 = Adafruit_MAX31865(MAX2CS);

#define RREF 400.0
#define RNOMINAL 100



//gps defines
#define DEFAULT_DEVICE_ADDRESS 0x3A
#define DEFAULT_DEVICE_PORT 0xFF
#define I2C_DELAY 1

#define RESET_PIN 7

/*
adresses
bmi088 accel = 0x18
bmi088 gyro = 0x68

bmp = 0x76

adxl = 0x53

mag = 0x1c
*/


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
    void read(int oversampling = 5,int hitltesting = 0,int hitlindex = 0);
    IMUdata readraw(int oversampling = 5,int interval = 100);

    //calibration matrices (todo)
    Vector3d calibrationpos;
    Vector3d calibrationneg;

    // calibration matrices
    Vector3d bcal;
    Matrix3d acal;


};
// constructer, assigns values to bcal and acal
IMU::IMU(){
        
        bcal <<-0,0,0;

        acal << 1,0,0
        ,0,1,0,
        0,0,1;
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
void IMU::read(int oversampling, int hitltesting,int hitlindex){
    IMUdata _data;
    Vector3d accel;
    Vector3d gyro;

    accel << 0,0,0;
    gyro << 0,0,0;

    // Serial.printf("\n>NAVspot: %f \n", 1.51);
    rp2040.idleOtherCore();
    if (hitltesting)
    {
        //time,baro_altitude,accl_z,accl_y,accl_x,gps_altitude,gyro_roll,gyro_pitch,gyro_yaw
        accel.x() = hitldata[hitlindex][4];
        accel.y() = hitldata[hitlindex][2];
        accel.z() = hitldata[hitlindex][3];
        float convertorads = (PI/180);
        gyro.x() = hitldata[hitlindex][7]*convertorads;
        gyro.y() = hitldata[hitlindex][8]*convertorads;
        gyro.z() = hitldata[hitlindex][6]*convertorads;
    }
    
    else
    {    
    
    for (int i = 0; i < oversampling; i++)
    {
        
        accelunit.readSensor();
        gyrounit.readSensor();
        
    
        accel.x() += accelunit.getAccelX_mss();
        accel.y() += accelunit.getAccelZ_mss();
        accel.z() += accelunit.getAccelY_mss();
        //Serial.printf("new accelmss z : %f \n",accel.z());

        gyro.x() += gyrounit.getGyroX_rads();

        gyro.y() += gyrounit.getGyroZ_rads();
        gyro.z() += gyrounit.getGyroY_rads();

        delayMicroseconds(1);
        gyro.x() < -73786 || gyro.x() > 73786 ? gyro.x() = data.gyro.x : gyro.x() = gyro.x();
        gyro.y() < -73786 || gyro.y() > 73786 ? gyro.y() = data.gyro.x : gyro.y() = gyro.y();
        gyro.z() < -73786 || gyro.z() > 73786 ? gyro.z() = data.gyro.x : gyro.z() = gyro.z();

        accel.x() < -7378 || accel.x() > 7378 ? accel.x() = data.accel.x : accel.x() = accel.x();
        accel.y() < -7378 || accel.y() > 7378 ? accel.y() = data.accel.x : accel.y() = accel.y();
        accel.z() < -7378 || accel.z() > 7378 ? accel.z() = data.accel.x : accel.z() = accel.z();
        
    }
    rp2040.resumeOtherCore();
    //Serial.printf("\n>NAVspot: %f \n", 1.52);
    
    
    accel.x() /= oversampling;
    accel.y() /= oversampling;
    accel.z() /= oversampling;
    
    gyro.x() /= oversampling;
    gyro.y() /= oversampling;
    gyro.z() /= oversampling;

    // accel calibration
    accel = accel - bcal;

    accel = acal * accel;
    }

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

    //Serial.printf("\n>NAVspot: %f \n", 1.53);
    
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

class MAX31865
{
private:
    
public:
    MAX31865();
    ~MAX31865();
    int init();
    float read1(int printon);
    float read2(int printon);
};

MAX31865::MAX31865()
{
}

MAX31865::~MAX31865()
{
}


int MAX31865::init(){
    SPI.setRX(MISO);
    SPI.setTX(MOSI);
    SPI.setSCK(SCLK);
    int status = thermo.begin(MAX31865_2WIRE);
    Serial.printf("thermo1 status %d",status);
    status = thermo2.begin(MAX31865_2WIRE);
    Serial.printf("thermo2 status %d",status);

    thermo.autoConvert(true);
    thermo2.autoConvert(true);
    return status;
}


float MAX31865::read1(int printon){
    //uint16_t rtd = thermo.readRTD();

    if (printon)
    {
        //Serial.print("RTD value: "); Serial.println(rtd);
    // float ratio = rtd;
    // ratio /= 32768;
    // Serial.print("Ratio = "); Serial.println(ratio,8);
    // Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
    // Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

    // Check and print any faults
    uint8_t fault = thermo.readFault();
    if (fault) {
        Serial.print("Fault 0x"); Serial.println(fault, HEX);
        if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold"); 
        }
        if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold"); 
        }
        if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias"); 
        }
        if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
        }
        if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
        }
        if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage"); 
        }
        thermo.clearFault();
    }
    Serial.println();
    }
    
    
    return thermo.temperature(RNOMINAL, RREF);
}

float MAX31865::read2(int printon){
    //uint16_t rtd = thermo2.readRTD();

    if (printon)
    {
        //Serial.print("RTD value: "); Serial.println(rtd);
    // float ratio = rtd;
    // ratio /= 32768;
    // Serial.print("Ratio = "); Serial.println(ratio,8);
    // Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
    // Serial.print("Temperature = "); Serial.println(thermo2.temperature(RNOMINAL, RREF));

    // Check and print any faults
    uint8_t fault = thermo2.readFault();
    if (fault) {
        Serial.print("Fault 0x"); Serial.println(fault, HEX);
        if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Serial.println("RTD High Threshold"); 
        }
        if (fault & MAX31865_FAULT_LOWTHRESH) {
        Serial.println("RTD Low Threshold"); 
        }
        if (fault & MAX31865_FAULT_REFINLOW) {
        Serial.println("REFIN- > 0.85 x Bias"); 
        }
        if (fault & MAX31865_FAULT_REFINHIGH) {
        Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
        }
        if (fault & MAX31865_FAULT_RTDINLOW) {
        Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
        }
        if (fault & MAX31865_FAULT_OVUV) {
        Serial.println("Under/Over voltage"); 
        }
        thermo2.clearFault();
    }
    Serial.println();
    }
    

    
    return thermo2.temperature(RNOMINAL, RREF);
}



/*--------------------------------------------------------------------------------------*/

//class to store all code to interface with the high G accel 
class SERIALPORT{
public:

    bool sendtoplot = true;
    SERIALPORT();
    int init();
    int senddata(mpstate state,navpacket navstate);

};

SERIALPORT::SERIALPORT(){}

//code to initilze the serial moniter
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

// send telemetr yover serial to teleplot, its ugly and I hate it but idk a better + more readable way would be 
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
            Serial.printf(">gyro x: %f \n",navstate.r.imudata.gyro.x*(180/M_PI) );
            Serial.printf(">gyro y: %f \n",navstate.r.imudata.gyro.y*(180/M_PI));
            Serial.printf(">gyro z: %f \n",navstate.r.imudata.gyro.z*(180/M_PI));
            Serial.printf(">state : %d \n",state.r.state);
            Serial.printf(">temp1: %f \n",navstate.r.temp1);
            Serial.printf(">temp2: %f \n",navstate.r.temp2);
            return 0;
        }
        

        return 0;
    }
/*-------------------------------------------------------------------------------------------*/

#endif // LYRAV2SENSORSLIB