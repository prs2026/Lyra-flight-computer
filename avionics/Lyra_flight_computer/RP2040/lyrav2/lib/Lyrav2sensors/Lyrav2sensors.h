#if !defined(Lyrav2sensors)
#define Lyrav2sensors
#include <Arduino.h>
#include <Wire.h>

#include <macros.h>

#include <BMI088.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_LIS3MDL.h>

#include <generallib.h>
#include <E220.h>
// #include <e22.h>


/* accelunit object */
Bmi088Accel accelunit(Wire1,0x18);
/* gyrounit object */
Bmi088Gyro gyrounit(Wire1,0x68);

Adafruit_BMP3XX bmp;
Adafruit_LIS3MDL mdl;

Stream &radioserial = (Stream &)Serial1;
                    //  m0         m1     aux
E220 ebyte(&radioserial,SERVO4,26,SERVO3);

                // aux m1
// e22 ebytesimple(SERVO3,UART0_RX);

const float SEALEVELPRESSURE = 1023.3;



uint8_t scani2c(bool printout){
    byte error, address;
    int nDevices;
     printout ? Serial.print("Scanning..")  : 0;
    
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire1.beginTransmission(address);
        error = Wire1.endTransmission();
        if (error == 0)
        {
        printout ? Serial.print("I2C device found at address 0x") : 0;
        if (address<16)
            Serial.print("0");
        printout ? Serial.print(address,HEX) : 0;
        printout ? Serial.println("  !") : 0;
    
        nDevices++;
        }
        else if (error==4)
        {
        printout ? Serial.print("Unknown error at address 0x") : 0;
        if (address<16)
            printout ? Serial.print("0") : 0;
            printout ? Serial.println(address,HEX) : 0;
        }    
    }
    if (nDevices == 0){
        printout ? Serial.println("No I2C devices found\n") : 0;
        return 1;
    }
    else
        printout ? Serial.println("done\n") : 0;
    
    return 0;
}
/*
Quaterniond intergrategyros(Quaterniond prevstate){
    Quaterniond result;

    return result;
}
*/
class IMU{

Vector3d bcal;
Matrix3d acal;

IMUdata prevdata;

float gyroal = 0.3;
float accelal = 0.3;

public:
    IMU(){
        
        bcal << 0.02305,0.0063,0.003;

        acal << 1.004078865,0,0,
                0,1.003614894,0,
                0,0,1.007061535;
        
    };
    IMUdata data;


    

    int init(){
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

    void read(int oversampling = 32){
        IMUdata _data;
        Vector3d accel;
        Vector3d gyro;

        accel << 0,0,0;
        gyro << 0,0,0;
        
        for (int i = 0; i < oversampling; i++)
        {
            accelunit.readSensor();
            gyrounit.readSensor();

            accel.x() += accelunit.getAccelZ_mss();
            accel.y() += accelunit.getAccelY_mss();
            accel.z() += -accelunit.getAccelX_mss();

            gyro.x() += -gyrounit.getGyroZ_rads();
            gyro.y() += -gyrounit.getGyroY_rads();
            gyro.z() += gyrounit.getGyroX_rads(); // when the radians to degrees calculation of 180/PI is done at runtime, it breaks but this works so 

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


        _data.accel = vector3tofloat(accel);
        _data.gyro = vector3tofloat(gyro);

        _data.temp = accelunit.getTemperature_C();

        data = _data;
        prevdata = _data;
        
        return;
        
    }

};


class BARO
{
float prevverticalvel[5];
float prevalt;

int address = 0;
uint64_t prevtime;

float lpfal = 0.7;
float lpfalv = 0.7;
// float hpfcutoff = 5;
// float hpfstate = 0;

public:
    BARO(){


    };
    BAROdata data;
    double padalt = 0;


    int getpadoffset(int samplesize = 1){
        double _padalt = 0;
        // for (int i = 0; i < samplesize; i++)
        // {
            _padalt += bmp.readAltitude(SEALEVELPRESSURE);
            delayMicroseconds(100);
        // }

        _padalt /= samplesize;
        Serial.printf("new pad offset: %f\n",_padalt);
        //MP.logtextentry("BMP new pad offset: ",float(_padalt));
        padalt = _padalt;
        data.maxrecordedalt = 0;
        return 0;
    }

    int init(){
        if (!bmp.begin_I2C(0x76,&Wire1))
        {
            Serial.println("BMP init failure");
            //MP.logtextentry("BMP init fail");
            return 1;
        }
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setOutputDataRate(BMP3_ODR_100_HZ);
        getpadoffset();
        Serial.println("BMP init success");
        //MP.logtextentry("BMP init fail");
        return 0;
    }
    void readsensor(){
        BAROdata _data;
        bmp.performReading();

        float timestep = (micros() - prevtime)/1e6;
        // float hpfgain = hpfcutoff / (2* M_PI * 1/timestep);

        _data.altitude = 44330.0 * (1.0 - pow((bmp.pressure/100.0F) / SEALEVELPRESSURE, 0.1903));

        _data.altitude = lpfal*prevalt + (1-lpfal)*_data.altitude;
        // _data.altitude = _data.altitude - hpfstate;

        // hpfstate += hpfgain * _data.altitude;


        _data.altitudeagl = _data.altitude-padalt;
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

};

class MAG{

float bcali[3] = {-11.1517,-51.38845,0};

float acali[3][3] = {
    {0.4181084228,0,0},
    {0,0.3914369911,0},
    {0,0,1}};



public:
    MAG(){};
    MAGdata data;
    int init(){
        if (!mdl.begin_I2C(0x1C,&Wire1))
        {
            Serial.println("MAG init fail");
            return 1;
        }

        Serial.println("MAG init success");
    return 0;
    }

    int read(){
        mdl.read();

        data.gauss.x = mdl.x;
        data.gauss.y = mdl.y;
        data.gauss.z = mdl.z;

        sensors_event_t event;
        
        mdl.getEvent(&event);

        data.utesla.x = event.magnetic.x;
        data.utesla.y = event.magnetic.y;
        data.utesla.z = event.magnetic.z;

        float currmeas[3] = {data.utesla.x-bcali[0],data.utesla.y-bcali[1],data.utesla.z-bcali[2]};
        //Serial.printf("%f, %f, %f gainadj: %f, %f, %f ",_data.accel.x,_data.accel.y,_data.accel.z,currmeas[0],currmeas[1],currmeas[2]);
        data.utesla.x = acali[0][0]*currmeas[0]+acali[1][0]*currmeas[1]+acali[2][0]*currmeas[2];
        data.utesla.y = acali[0][1]*currmeas[0]+acali[1][1]*currmeas[1]+acali[2][1]*currmeas[2];
        data.utesla.z = acali[0][2]*currmeas[0]+acali[1][2]*currmeas[1]+acali[2][2]*currmeas[2];


        //data.headingdeg = atan2(data.utesla.y,data.utesla.x)*(180/PI);
        return 0;
    } 


};

class SERIALPORT{

public:

    bool sendtoplot = true;

    SERIALPORT(){

    };

    int init(){
            Serial.begin(115200);

            uint32_t serialstarttime = millis();
            while (!Serial && millis() - serialstarttime < 3000);
            delay(100);

            if (Serial)
            {
                Serial.println("\n\nMP Serial init");
                //MP.logtextentry("MP serial init");
                return 0;
            }
            else{
                return 1;
            }
    }

    
    int senddata(mpstate state,navpacket navstate){
        if (sendtoplot)
        {
            Serial.printf(
            ">MP uptime: %d \n" 
            ">NAV uptime: %d \n" 
            ">MP errorflag %d \n" 
            ">NAV errorflag %d \n" 
            ">accel x: %f \n" 
            ">accel y: %f \n"
            ">accel z: %f \n"
            ">accelworld x: %f \n" 
            ">accelworld y: %f \n"
            ">accelworld z: %f \n"  
            ">gyro x: %f \n" 
            ">gyro y: %f \n"
            ">gyro z: %f \n"
            ">altitude: %f \n" 
            ">verticalvel: %f \n"
            ">filtered vvel: %f \n"
            ">mag x: %f \n" 
            ">mag y: %f \n" 
            ">mag z: %f \n"
            // ">magraw x: %f \n"
            // ">magraw y: %f \n"
            // ">magraw z: %f \n"
            ">orientation pitch: %f \n"
            ">orientation yaw: %f \n"
            ">orientation roll: %f \n"

            ">orientation w: %f \n"
            ">orientation x: %f \n"
            ">orientation y: %f \n"
            ">orientation z: %f \n"

            ">maxrecorded alt: %f \n"
            ">filtered alt: %f \n"
            ">state : %d \n"
            ">altitudeagl : %f \n"
            ">varience alt : %f \n"
            ">varience vvel : %f \n"
            ">battery vol : %f \n"
            ">baro temp : %f \n",
            state.r.uptime
            ,navstate.r.uptime

            , state.r.errorflag
            , navstate.r.errorflag

            ,navstate.r.imudata.accel.x
            ,navstate.r.imudata.accel.y
            ,navstate.r.imudata.accel.z

            ,navstate.r.accelworld.x
            ,navstate.r.accelworld.y
            ,navstate.r.accelworld.z

            ,navstate.r.imudata.gyro.x*(180/M_PI)
            ,navstate.r.imudata.gyro.y*(180/M_PI)
            ,navstate.r.imudata.gyro.z*(180/M_PI)

            ,navstate.r.barodata.altitude
            ,navstate.r.barodata.verticalvel
            ,navstate.r.filtered.vvel

            ,navstate.r.magdata.utesla.x
            ,navstate.r.magdata.utesla.y
            ,navstate.r.magdata.utesla.z

            // ,navstate.r.magdata.gauss.x
            // ,navstate.r.magdata.gauss.y
            // ,navstate.r.magdata.gauss.z

            ,navstate.r.orientationeuler.x*(180/M_PI)
            ,navstate.r.orientationeuler.y*(180/M_PI)
            ,navstate.r.orientationeuler.z*(180/M_PI)

            ,navstate.r.orientationquat.w
            ,navstate.r.orientationquat.x
            ,navstate.r.orientationquat.y
            ,navstate.r.orientationquat.z

            ,navstate.r.barodata.maxrecordedalt
            ,navstate.r.filtered.alt
            ,state.r.state
            ,navstate.r.barodata.altitudeagl
            ,navstate.r.uncertainty.alt
            ,navstate.r.uncertainty.vvel
            ,state.r.batterystate
            ,navstate.r.barodata.temp
                );
                // this is ugly, but better than a million seperate prints
            return 0;
        }
        else
        {
            Serial.printf("%f,%f,%f \n",navstate.r.orientationeuler.x,navstate.r.orientationeuler.y,navstate.r.orientationeuler.z);
        }
        
        

        return 0;
    }



};

class RADIO{

    public:
    int type = NRF24;

    int NRF24init(){
            SPI.end();
            SPI.setRX(SPI0_MISO);
            SPI.setTX(SPI0_MOSI);
            SPI.setSCK(SPI0_SCLK);
            SPI.begin();
            //Serial.println("radio init start");

            int error = radio.begin(&SPI);
            if (!error)
            {
                Serial.println("NRF24 init fail");
                error = radio.isChipConnected();

                if (!error)
                {
                    Serial.println("NRF24 not connected");
                    return 1;
                }

            }
            Serial.println("NRF24 init success");

    

            //radio.setPALevel(RF24_PA_MAX);
            //radio.setAutoAck(true);
            //radio.setRetries(10,15);
            //radio.setDataRate(RF24_250KBPS);

            radio.openWritingPipe(radioaddress[1]);
            radio.openReadingPipe(1,radioaddress[0]);

            uint32_t radiostarttime = millis();

            bool sucess = false;

            while ((millis() - radiostarttime) < 2000)
            {
                if (radiocommcheck() == 0)
                {
                    sucess == true;
                    break;
                }
                //Serial.println("radio handshake fail");
            }
            if (sucess)
            {
               
                Serial.println("radio handshake timeout");
                return 1;
            }
            Serial.println("radio handshake complete");
            return 0;

            
        }


    int E22init(){
        //Serial.println("starting e22 init");
    
        Serial1.end();
        Serial1.setRX(SERVO2);
        Serial1.setTX(SERVO1);
        Serial1.begin(9600);

        //Serial.println("Serial inited");
    


        uint32_t inittime = millis();    
        while (millis()-inittime < 1000)
        {
            if (ebyte.init())
            {
                Serial.println("radio init sucess");
                break;
            }
            Serial.println("radio init attempt fail");
            
        }

        ebyte.setAddress(0xffff,true);
        ebyte.setPower(Power_21,true);
        ebyte.setChannel(68,true);
        ebyte.setNetID(0,true);
        ebyte.setBaud(UDR_9600,true);
        ebyte.setSubPacketSize(SPS_64,true);
        ebyte.setAirDataRate(ADR_0300,true);
        ebyte.setEncryptionKey(0,true);
        ebyte.setLBT(true,true);
        ebyte.printBoardParameters();
        return 0;
    }


    int begin(int radiotype = 0){
        int error;
        // error = NRF24init();
        // if (error == 0){ return 0; }

        error = E22init();
        return 1;
    }

    int sendpacket(telepacket packet){
        Serial1.write(packet.data,sizeof(packet.data));

        //uint8_t testpacket[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};
        //Serial1.write(testpacket,sizeof(testpacket));

        if (Serial1.available() && Serial1.read() == 0xFF)
        {
            ebyte.setRadioMode(MODE_NORMAL);
        }
        
        return 0;
    }
};

class ADC{

public:
    ADC(){};

    float battvoltage = 0;
    
    void setuppins(){
        pinMode(BATT_SENSE, INPUT);
        return;
    }

    void readbatt(){
        float buffer = analogRead(BATT_SENSE);
        battvoltage = map(buffer,0,4096,0,15);
    }

};





#endif // Lyrav2sensors