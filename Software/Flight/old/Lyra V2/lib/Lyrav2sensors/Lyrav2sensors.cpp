#include <Lyrav2sensors.h>

IMU::IMU(){
        
        bcal << 0.02305,0.0063,0.003;

        acal << 1.004078865,0,0,
                0,1.003614894,0,
                0,0,1.007061535;
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

void IMU::read(int oversampling = 32){
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
    
int BARO::getpadoffset(int samplesize = 1){
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
    Serial.println("BMP init success");
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