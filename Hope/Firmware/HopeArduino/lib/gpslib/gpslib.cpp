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
        Serial.print(byte);
        
        // if (byte == '\n')
        // {   
        //     break;
        // }
        gpsparser.encode(byte);
        delay(2);
    }

    Serial1.flush();
    
    return 0;
}

    // //
    // Serial.printf("time is valid? %d hour/minute/second %d,%d,%d \n",gpsparser.time.isUpdated(),gpsparser.time.hour(),gpsparser.time.minute(),gpsparser.time.second());
    // Serial.printf("pos is valid? %d lat %d, lon %d \n",gpsparser.location.isUpdated(),gpsparser.location.lat(),gpsparser.location.lng());
    // Serial.printf("sats in view? %d updated? %d\n",gpsparser.satellites.value(),gpsparser.satellites.isUpdated());