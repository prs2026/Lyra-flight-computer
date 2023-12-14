#include <Arduino.h>
#include "generallib.h"
#include "navcore.h"
#include "mpcore.h"

MPCORE MP;

bool dataismoved = false;


void setup() { // main core setup
    MP.setuppins();
    MP.beep();
    MP.setled(BLUE);
    MP.initperipherials();
    MP.logtextentry("\n\n MP init");

    // MP.handshake();

    MP.flashinit();
    MP.initsd();
    MP.movedata();
    MP.logdata();
    MP.logcurrentstate();
    
    waitfornextfifo();
    


    if (NAV._sysstate.r.errorflag * MP._sysstate.r.errorflag != 1)
    {
        MP.ledcolor = BLUE;
        //Serial.println("core error");
    }
    else if (NAV._sysstate.r.errorflag * MP._sysstate.r.errorflag < 0){
        MP.ledcolor = RED;
    }
    else{
        MP.ledcolor = GREEN;
        //Serial.println("cores good");
    }
    

    MP._sysstate.r.uptime = millis();


    Serial.print("MP boot complete error code: ");
    Serial.println(MP._sysstate.r.errorflag);

    //MP.fetchnavdata();
    
    Serial.print("NAV boot complete, error code :");
    Serial.println(NAV._sysstate.r.errorflag);
    
    //MP.readdata();

    //MP.beep();
}

void setup1() { // nav core setup
    // NAV.handshake();
    delay(200);
    NAV.initi2c();
    NAV.sensorinit();
    // navpacket initpacket;
    // initpacket.r.errorflag = NAV._sysstate.r.errorflag;
    // NAV.sendpacket(initpacket);
    baro.getpadoffset();
    NAV.getsensordata();
    NAV.KFinit();
    rp2040.fifo.push(0xAB);
}

void loop() { // main core loop
    int eventsfired = 0;

    if (millis() - MP.prevtime.logdata >= MP.intervals[MP._sysstate.r.state].logdata)
    {
        if (MP.sendserialon && MP.sendtoteleplot)
        {
            Serial.print(">shouldlog: 1 \n");
        }
    }

    if (millis() - MP.prevtime.logdata >= MP.intervals[MP._sysstate.r.state].logdata)
    {
        //uint32_t prevlogmicros = micros();
        MP.logdata();
        if (MP.sendserialon && MP.sendtoteleplot)
        {
            Serial.printf(">lograte: %f \n",1000/float((millis()-MP.prevtime.logdata)));
        }
        MP.prevtime.logdata = millis();
        eventsfired += 2;
        //Serial.printf("logging  took: %d \n",micros() - prevlogmicros);
    }
    
    
    if (millis()- MP.prevtime.detectstatechange >= MP.intervals[MP._sysstate.r.state].detectstatechange)
    {
        MP.changestate();
        MP.prevtime.detectstatechange = millis();
    }

    if (MP.sendserialon & millis() - MP.prevtime.serial >= MP.intervals[MP._sysstate.r.state].serial)
    {
        port.senddata(MP._sysstate,NAV._sysstate);
        MP.prevtime.serial = millis();
        //eventsfired += 20;
    }
    


    if (millis()- MP.prevtime.led >= MP.intervals[MP._sysstate.r.state].led)
    {
        MP.ledstate ? MP.setled(MP.ledcolor) : MP.setled(OFF);
        MP.ledstate =! MP.ledstate;
        MP.prevtime.led = millis();
        
    }

    if (millis()- MP.prevtime.beep >= MP.intervals[MP._sysstate.r.state].beep)
    {
        switch (MP._sysstate.r.state)
        {
        case 0:
            MP.beep(600);
            break;

        case 1: // ready to launch
            MP.beep(4000);
            break;

        case 2: // powered ascent
            MP.beep(6000);
            break;

        case 3: // unpowered descent
            MP.beep(6000);
            break;

        case 4: // under canopy
            MP.beep(6000);
            break;

        case 5: // ready to land
            MP.beep(2000);
            break;
        
        case 6: // landed
            MP.beep(4000);
            delay(50);
            MP.beep(4000);
            break;
        
        default:
            break;
        }
        MP.prevtime.beep = millis();
        eventsfired += 10;
    }
    
    
    if (Serial.available())
    {
        char buf = Serial.read(); 
        int i;
        Serial.printf("echo: %c dec: %d \n",buf,buf);
        MP.parsecommand(buf);

    }

    


    
    if ((millis() - MP.prevtime.sendtelemetry >= MP.intervals[MP._sysstate.r.state].sendtelemetry) && MP._sysstate.r.errorflag %19 != 0)
    {
        uint32_t prevtelemmicros = micros();
        MP.sendtelemetry();
        MP.prevtime.sendtelemetry = millis();
        eventsfired += 4;
        //Serial.printf("telemetry sending took: %d \n",micros() - prevtelemmicros);
    }

    if (radio.available())
    {
        char buf;
        radio.read(&buf,1);
        MP.parsecommand(buf);
    }
    
    if ( dataismoved == false)
    {
        if (MP._sysstate.r.state == 6 && millis() - MP.landedtime >= 5000){
            dataismoved = true;
            MP.movedata();
        }
    }
    
    
    
    MP._sysstate.r.uptime = millis();
    if (MP.sendserialon && MP.sendtoteleplot)
    {
        Serial.printf(">looptime: %f \n", float(micros() - MP.prevtime.loop)/1000);
        Serial.printf(">eventstranspired: %d \n", eventsfired);
        Serial.print(">shouldlog: 0 \n");
    }
    
    MP.prevtime.loop = micros();
    MP._sysstate.r.state >= 1 ? MP.missionelasped = millis() - MP.liftofftime : MP.missionelasped = 0, MP.landedtime = millis();

    if (Serial1.available())
    {
        Serial.println("newradiomessage");
        while (Serial1.available() > 0)
        {
            uint8_t readbuf = Serial1.read();
            Serial.printf("0x%x, ",readbuf);
            printBin(readbuf);
            Serial.printf(", %d\n",readbuf);
        }
        
    }
    
}


void loop1() { // nav core loop
    if (MP._sysstate.r.state == 0)
    {
        NAV.alpha = 0.8;
        //Serial.printf(">navalpha: 0.8\n");
    }
    else
    {
        NAV.alpha = 0.98;
        //Serial.printf(">navalpha: 0.98\n");
    }
    
    NAV.getsensordata();
    NAV.KFpredict();

    if (millis() - NAV.prevtime.kfupdate >= 300)
    {
        NAV.KFupdate();
        NAV.prevtime.kfupdate = millis();
    }
    

    
    
    

    // if ((millis() - NAV.prevtime.sendpacket) >= 50)
    // {
    //     int inbuf = rp2040.fifo.available();
    //     int error = NAV.sendpacket(NAV._sysstate);
    //     if (error > 0)
    //     {
    //         //Serial.printf("packet send failure at iteration %d and timestamp %d with %d bytes already in fifo \n"
    //         //,error,NAV._sysstate.r.uptime,inbuf);
    //     }
    //     else
    //     {
    //         //Serial.print("packet sent on iteration ");
    //         //Serial.print(error);
    //     }
        
    //     NAV.prevtime.sendpacket = millis();
        
    // }
    
    NAV._sysstate.r.uptime = millis();
}



    // if (rp2040.fifo.available())
    // {
    //     uint32_t gettingnavdata = micros();
    //     int _avalible = rp2040.fifo.available();
    //     int _error = MP.fetchnavdata();
    //     eventsfired += 1;
    //     //Serial.printf("fetching nav data took %d \n",micros() - gettingnavdata);
    //     //Serial.printf("recived packet at timestamp : %d with error %d and %d bytes in the fifo",MP._sysstate.r.uptime,_error,_avalible);
    // }