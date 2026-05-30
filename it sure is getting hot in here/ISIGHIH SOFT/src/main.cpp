#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#include "mpcore.h"

MPCORE MP;
Adafruit_USBD_MSC usb_msc;

static int32_t mscReadCallback(uint32_t lba, void *buffer, uint32_t bufsize)
{
    if (bufsize % 512 != 0)
    {
        return -1;
    }

    uint8_t *dst = (uint8_t *)buffer;
    uint32_t blockCount = bufsize / 512;
    for (uint32_t block = 0; block < blockCount; block++)
    {
        if (MP.readCsvExportSector(lba + block, dst + block * 512) != 0)
        {
            memset(dst + block * 512, 0, 512);
        }
    }

    return (int32_t)bufsize;
}

static int32_t mscWriteCallback(uint32_t lba, uint8_t *buffer, uint32_t bufsize)
{
    (void)lba;
    (void)buffer;
    (void)bufsize;
    return -1;
}

static void mscFlushCallback(void)
{
}

static bool mscWritableCallback(void)
{
    return false;
}

static void enterMassStorageMode()
{
    Serial.println("MSC export mode requested");
    Serial.println("Building CSV export");
    Serial.flush();

    if (MP.buildCsvExportFlash() != 0)
    {
        Serial.println("MSC export build failed");
        MP.usbMassStorageMode = false;
        MP.exportReady = false;
        return;
    }

    Serial.println("Switching USB to mass storage");
    Serial.flush();
    delay(100);

    TinyUSBDevice.detach();
    delay(50);

    Serial.end();

    usb_msc.setCapacity(MP.exportSectorCount, 512);
    usb_msc.setUnitReady(true);
    usb_msc.begin();

    MP.exportReady = true;

    TinyUSBDevice.attach();
}

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

    Serial.println("mpcore out of setup");

    usb_msc.setID("LYRA", "ISIGHIHCSV", "1.0");
    usb_msc.setCapacity(MSC_BLOCK_COUNT, 512);
    usb_msc.setReadWriteCallback(mscReadCallback, mscWriteCallback, mscFlushCallback);
    usb_msc.setWritableCallback(mscWritableCallback);
    usb_msc.setUnitReady(false);
    
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
    NAV.getsensordata(MP.sendtoteleplot);
    NAV.ready = 1;
}

void loop() { // main core loop
    int eventsfired = 0;

    if (MP.usbMassStorageMode)
    {
        if (!MP.exportReady)
        {
            enterMassStorageMode();
        }

        TinyUSBDevice.task();
        return;
    }

    MP.changestate();
    //MP.checkforpyros();
    //Serial.printf(">loops: 1 \n");
    // if (millis() - MP.prevtime.logdata >= MP.intervals[MP._sysstate.r.state].logdata){
    //     if (MP.sendserialon && MP.sendtoteleplot)
    //     {
    //         Serial.print(">shouldlog: 1 \n");
    //     }
    // }
    
    if (millis() - MP.prevtime.logdata >= MP.intervals[MP._sysstate.r.state].logdata && NAV.newdata){
        uint32_t prevlogmicros = micros();
        //Serial.print(">Writing somewhere: 1 \n");
        if (MP._sysstate.r.state == 0 || MP._sysstate.r.state == 5)
        {
            MP.logtobuf();
        }
        else
        {
            MP.logdata(MP._sysstate,NAV._sysstate);
        }
        //NAV.newdata = 0;
        // if (MP.sendserialon && MP.sendtoteleplot)
        // {
        //     Serial.printf(">lograte: %f \n",1000/float((millis()-MP.prevtime.logdata)));
        // }
        //Serial.print(">Writing somewhere: 2 \n");
        MP.prevtime.logdata = millis();
        //eventsfired += 2;
        //MP.sendserialon ? Serial.printf(">loggingtime: %d \n",micros() - prevlogmicros) : 1==1;
    }

    //
    if (MP.sendserialon & millis() - MP.prevtime.serial >= MP.intervals[MP._sysstate.r.state].serial){
        uint32_t prevserialmicros = micros();
        port.senddata(MP._sysstate,NAV._sysstate);
        MP.prevtime.serial = millis();
        MP.sendserialon ? Serial.printf(">porttime: %d \n",micros() - prevserialmicros) : 1==1;
    }


    if (millis()- MP.prevtime.led >= MP.intervals[MP._sysstate.r.state].led){
        MP.ledstate ? MP.setled(1) : MP.setled(0);
        MP.ledstate =! MP.ledstate;
        MP.prevtime.led = millis();
    }

    if (Serial.available()){
        char buf = Serial.read(); 
        int i;
        Serial.printf("echo: %c dec: %d \n",buf,buf);
        MP.parsecommand(buf);
    }

    // if (MP._sysstate.r.uptime > 10000 && cameraon == false){
    //     P1.fire();
    // }

    MP.prevtime.loop = micros();
    MP._sysstate.r.state >= 1 ? MP.missionelasped = millis() - MP.liftofftime : MP.missionelasped = 0, MP.landedtime = millis();

    NAV.state = MP._sysstate.r.state;

    MP._sysstate.r.uptime = millis();
    //Serial.printf(">loops: 2 \n");



}


void loop1() { // nav core loop
    if (MP.usbMassStorageMode)
    {
        return;
    }

    MP._sysstate.r.state == 0 ? NAV.useaccel = 1 : NAV.useaccel = 0;
    //Serial.printf("\n>NAVspot: %d \n", 1);
    
    NAV.prevtime.getdata = micros();
    NAV.getsensordata(MP.sendtoteleplot);
    NAV.newdata = 1;
    //Serial.print("nav 1\n");
    if (MP.sendserialon && MP.sendtoteleplot)
    {
            Serial.printf(">sensordatatime: %f \n",float(micros()-NAV.prevtime.getdata)/1000);
            
            NAV.prevtime.predictkf = micros();
    }
    //Serial.printf("\n>NAVspot: %d \n", 2);
    
    if (MP.sendserialon & millis() - MP.prevtime.serial >= MP.intervals[MP._sysstate.r.state].serial)
    {
        Serial.printf(">navlooprate: %f \n", 1/(float(micros() - NAV.prevtime.looptime)/1e6));
        
        MP.prevtime.serial = millis();
    }
    NAV.prevtime.looptime = micros();
    NAV._sysstate.r.uptime = millis();
    //Serial.printf("\n>NAVspot: %d \n", 3);
}
