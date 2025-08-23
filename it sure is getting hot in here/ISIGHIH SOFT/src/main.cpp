#include <Arduino.h>

#include "mpcore.h"

MPCORE MP;

bool dataismoved = false;

uint32_t brkout1statustime = 0;

bool cameraon = false;

void setup() { // main core setup
    MP.setuppins();
    MP.beep();
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

    //Camera init special
    P1.timeout = 1.08e7;


    // Serial.print("MP boot complete error code: ");
    // Serial.println(MP._sysstate.r.errorflag);
    
    // Serial.print("NAV boot complete, error code :");
    // Serial.println(NAV._sysstate.r.errorflag);

    // MP.beep(4000,200);
    // MP.beep(4500,200);
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
    NAV.ready = 1;
}

void loop() { // main core loop
    int eventsfired = 0;
    MP.changestate();
    //MP.checkforpyros();

    // if (millis() - MP.prevtime.logdata >= MP.intervals[MP._sysstate.r.state].logdata){
    //     if (MP.sendserialon && MP.sendtoteleplot)
    //     {
    //         Serial.print(">shouldlog: 1 \n");
    //     }
    // }
    
    if (millis() - MP.prevtime.logdata >= MP.intervals[MP._sysstate.r.state].logdata && NAV.newdata){
        uint32_t prevlogmicros = micros();
        if (MP._sysstate.r.state == 0 || MP._sysstate.r.state == 5)
        {
            MP.logtobuf();
        }
        else
        {
            MP.logdata(MP._sysstate,NAV._sysstate);
        }
        NAV.newdata = 0;
        // if (MP.sendserialon && MP.sendtoteleplot)
        // {
        //     Serial.printf(">lograte: %f \n",1000/float((millis()-MP.prevtime.logdata)));
        // }
        MP.prevtime.logdata = millis();
        eventsfired += 2;
        //MP.sendserialon ? Serial.printf(">loggingtime: %d \n",micros() - prevlogmicros) : 1==1;
    }


    if (MP.sendserialon & millis() - MP.prevtime.serial >= MP.intervals[MP._sysstate.r.state].serial){
        uint32_t prevserialmicros = micros();
        Serial.printf(">brk1: %d \n",digitalRead(BRKOUT1));
        Serial.printf(">brk1time: %d \n",brkout1statustime);
        port.senddata(MP._sysstate,NAV._sysstate);
        MP.prevtime.serial = millis();
        MP.sendserialon ? Serial.printf(">porttime: %d \n",micros() - prevserialmicros) : 1==1;
    }


    if (millis()- MP.prevtime.led >= MP.intervals[MP._sysstate.r.state].led){
        MP.ledstate ? MP.setled(1) : MP.setled(0);
        MP.ledstate =! MP.ledstate;
        MP.prevtime.led = millis();
    }

    if (millis()- MP.prevtime.beep >= MP.intervals[MP._sysstate.r.state].beep){   
        if (!(MP._sysstate.r.uptime - brkout1statustime > 500))
        {
            MP.beep(5000);
            delay(10);
            MP.beep(5500);
        }

        if (MP._sysstate.r.state == 0)
        {
            //MP.beepcont();
        }
        else
        {
            MP.beep(MP.freqs[MP._sysstate.r.state]);
        }
        
        MP.prevtime.beep = millis();
        eventsfired += 10;
    }

    if (Serial.available()){
        char buf = Serial.read(); 
        int i;
        Serial.printf("echo: %c dec: %d \n",buf,buf);
        MP.parsecommand(buf);
    }

    if (MP._sysstate.r.uptime > 10000 && cameraon == false){
        P1.fire();
    }

    MP.prevtime.loop = micros();
    MP._sysstate.r.state >= 1 ? MP.missionelasped = millis() - MP.liftofftime : MP.missionelasped = 0, MP.landedtime = millis();

    NAV.state = MP._sysstate.r.state;

    MP._sysstate.r.uptime = millis();
    //Serial.printf("loops");
}


void loop1() { // nav core loop
    MP._sysstate.r.state == 0 ? NAV.useaccel = 1 : NAV.useaccel = 0;
    
    NAV.prevtime.getdata = micros();
    NAV.getsensordata(MP.sendtoteleplot);
    NAV.newdata = 1;
    //Serial.print("nav 1\n");
    if (MP.sendserialon && MP.sendtoteleplot)
    {
            Serial.printf(">sensordatatime: %f \n",float(micros()-NAV.prevtime.getdata)/1000);
            
            NAV.prevtime.predictkf = micros();
    }
    NAV.KFrun();
    //Serial.printf(">kfpredicttime: %f \n",float(micros()-NAV.prevtime.predictkf)/1000);
    //Serial.print("nav 2\n");
    if (millis() - NAV.prevtime.kfupdate >= 100)
    {
        NAV.prevtime.updatekf = micros();
        if (NAV.useaccel == 1)
    {
        NAV._sysstate.r.orientationquat = NAV.adjustwithaccel(0.1);
    }
        NAV.prevtime.kfupdate = millis();
        
        //Serial.printf(">kfupdatetime: %f \n",float(micros()-NAV.prevtime.updatekf)/1000);
    }

    //Serial.print("nav 3\n");
    
    if (MP.sendserialon & millis() - MP.prevtime.serial >= MP.intervals[MP._sysstate.r.state].serial)
    {
        Serial.printf(">navlooprate: %f \n", 1/(float(micros() - NAV.prevtime.looptime)/1e6));
        
        MP.prevtime.serial = millis();
    }
    NAV.prevtime.looptime = micros();
    NAV._sysstate.r.uptime = millis();
}