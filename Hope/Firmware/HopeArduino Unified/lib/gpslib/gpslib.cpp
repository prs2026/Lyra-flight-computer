#include <Arduino.h>
#include <basiclib.h>
#include <TinyGPS++.h>
#include <gpslib.h>

TinyGPSPlus gpsparser;

gpsinput::gpsinput(){
    return;
}

int gpsinput::checkformessages(){
    if (!Serial1.available())
    {
        return 1;
    }
    while (Serial1.available())
    { 
        //Serial.println("checking gps");
        char byte = Serial1.read();
        //Serial.print(byte); // for raw logging
        gpsparser.encode(byte);

    }

    if (gpsparser.time.isUpdated())
    {
        Serial.printf("time hour/minute/second %d,%d,%d \n",gpsparser.time.hour(),gpsparser.time.minute(),gpsparser.time.second());
    }
    if (gpsparser.location.isUpdated())
    {
        Serial.printf("lat %f, lon %f \n",gpsparser.location.lat(),gpsparser.location.lng());
    
    }
    if (gpsparser.satellites.isUpdated())
    {
        Serial.printf("sats in view %d\n",gpsparser.satellites.value());
    }

    

    
    return 0;
}

Vector3d gpsinput::getposition(){
    Vector3d position;
    if (gpsparser.location.isValid())
    {
        position.x() = gpsparser.location.lat();
        position.y() = gpsparser.location.lng();
        position.z() = gpsparser.altitude.meters();
    }
    else
    {
        position.x() = 0;
        position.y() = 0;
    }
    
    return position;
}

int gpsinput::hasfix(){
    if (gpsparser.location.isValid())
    {
        return 1;
    }
    else
    {
        return 0;
    }
}