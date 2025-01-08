#include <Arduino.h>

#include "mpcore.h"

MPCORE MP;

bool dataismoved = false;

uint32_t brkout1statustime = 0;

void setup() { // main core setup
    MP.setuppins();
    MP.beep();
    //gps.reset();
    delay(2000);
    MP.setled(1);
    MP.initperipherials();

    Serial.println("\n\nit be living yo");
    
    MP.ready = true;
    while (!NAV.ready)
    {
        delay(100);
    }

    MP._sysstate.r.uptime = millis();


    Serial.print("MP boot complete error code: ");
    Serial.println(MP._sysstate.r.errorflag);
    
    Serial.print("NAV boot complete, error code :");
    Serial.println(NAV._sysstate.r.errorflag);

    MP.beep(4000,200);
    MP.beep(4500,200);
    Serial.println("mpcore out of setup");
    
}

void setup1() { // nav core setup
    // NAV.handshake();
    while (MP.ready == false)
    {   
        delay(10);
    }
    Serial.println("\n\nnav init start");
    NAV.initi2c();
    NAV.sensorinit();
    NAV.getpadoffset();
    NAV.KFinit();
    NAV.getsensordata(MP.sendtoteleplot);
    NAV.getpadoffset();
    gps.reset();
    NAV.ready = 1;
}

void loop() { // main core loop
    int eventsfired = 0;
    MP.changestate();
    MP.checkforpyros();

    
        MP.checkforpyros();

    if (millis()- MP.prevtime.led >= MP.intervals[MP._sysstate.r.state].led)
    {
        MP.ledstate ? MP.setled(1) : MP.setled(0);
        MP.ledstate =! MP.ledstate;
        MP.prevtime.led = millis();
        
    }

    if (millis()- MP.prevtime.beep >= MP.intervals[MP._sysstate.r.state].beep)
    {   
        if (!(MP._sysstate.r.uptime - brkout1statustime > 500))
        {
            MP.beep(5000);
            delay(10);
            MP.beep(5500);
        }
        

        if (MP._sysstate.r.state == 0)
        {
            MP.beepcont();
        }
        else
        {
            MP.beep(MP.freqs[MP._sysstate.r.state]);
        }

        
        MP.prevtime.beep = millis();
        eventsfired += 10;
    }

    if ((millis() - MP.prevtime.sendtelemetry >= MP.intervals[MP._sysstate.r.state].sendtelemetry))
    {
        uint32_t prevtelemmicros = micros();
        MP.sendtelemetry();
        MP.prevtime.sendtelemetry = millis();
        eventsfired += 4;
        Serial.println("sendingtelem");
        MP.sendserialon ? Serial.printf(">telemetrytime: %d \n",micros() - prevtelemmicros): 1==1;
    }


    
    
    if (Serial.available())
    {
        char buf = Serial.read(); 
        int i;
        Serial.printf("echo: %c dec: %d \n",buf,buf);
        MP.parsecommand(buf);

    }
    MP.checkforpyros();
    Lora.request();
    //Lora.wait();
    // if (Lora.available())
    // {
    //     int buf = Lora.read();
    //     Serial.printf("recived %x from radio",buf);
    //     MP.parsecommand(buf);
    //     if (buf == 'P')
    //     {
    //         P1.fire();
    //     }
        
    // }

    MP.prevtime.loop = micros();
    MP._sysstate.r.state >= 1 ? MP.missionelasped = millis() - MP.liftofftime : MP.missionelasped = 0, MP.landedtime = millis();

    NAV.state = MP._sysstate.r.state;

    MP._sysstate.r.uptime = millis();
}


void loop1() { // nav core loop
}