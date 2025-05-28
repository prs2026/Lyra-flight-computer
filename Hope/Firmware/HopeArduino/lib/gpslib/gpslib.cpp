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
        char byte = Serial1.read();
        //Serial.print(byte);
        gpsparser.encode(byte);

        delay(2);
    }

    if (gpsparser.time.isUpdated())
    {
        Serial.printf("time hour/minute/second %d,%d,%d \n",gpsparser.time.hour(),gpsparser.time.minute(),gpsparser.time.second());
    }
    if (gpsparser.location.isUpdated())
    {
        Serial.printf("lat %d, lon %d \n",gpsparser.location.lat(),gpsparser.location.lng());
    
    }
    if (gpsparser.satellites.isUpdated())
    {
        Serial.printf("sats in view %d\n",gpsparser.satellites.value());
    }

    
    return 0;
}

    // //

            
        // if (byte == '\n')
        // {   
        //     break;
        // }
        //