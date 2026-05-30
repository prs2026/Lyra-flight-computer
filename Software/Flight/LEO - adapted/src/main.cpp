#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#include "mpcore.h"

MPCORE MP;
Adafruit_USBD_MSC usb_msc;

#if !defined(LYRA_CONTROL_PERIOD_US)
#define LYRA_CONTROL_PERIOD_US 10000
#endif

static constexpr uint32_t CONTROL_PERIOD_US = LYRA_CONTROL_PERIOD_US;
static constexpr uint8_t NAV_RATE_AVG_SAMPLES = 50;
static constexpr uint32_t TIMING_REPORT_PERIOD_MS = 500;

static bool flightLoggingState(uint8_t state)
{
    return state >= 1 && state <= 4;
}

static bool scheduledTickDue(uint32_t now, uint32_t &nextTick, uint32_t period)
{
    if ((int32_t)(now - nextTick) < 0)
    {
        return false;
    }

    do
    {
        nextTick += period;
    } while ((int32_t)(now - nextTick) >= 0);

    return true;
}

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
            return -1;
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

    //Camera init special
    P1.timeout = 1.08e7;


    Serial.print("MP boot complete error code: ");
    Serial.println(MP._sysstate.r.errorflag);
    
    Serial.print("NAV boot complete, error code :");
    Serial.println(NAV._sysstate.r.errorflag);

    MP.beep(4000,200);
    MP.beep(4500,200);
    Serial.println("mpcore out of setup");

    usb_msc.setID("LYRA", "FlightCSV", "1.0");
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
    NAV.getpadoffset();
    NAV.KFinit();
    NAV.getsensordata(MP.sendtoteleplot);
    NAV.getpadoffset();
    NAV.ready = 1;
}

void loop() { // main core loop
    static uint32_t nextFlightLogUs = micros() + CONTROL_PERIOD_US;
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
    MP.checkforpyros();

    uint8_t currentState = MP._sysstate.r.state;
    uint32_t nowUs = micros();
    if (flightLoggingState(currentState))
    {
        MP.flushpending = false;
        if (scheduledTickDue(nowUs, nextFlightLogUs, CONTROL_PERIOD_US) && NAV.newdata)
        {
            uint32_t prevlogmicros = micros();
            MP.logdata(MP._sysstate, NAV._sysstate);
            uint32_t logDurationUs = micros() - prevlogmicros;
            NAV.newdata = 0;
            MP.prevtime.logdata = millis();
            eventsfired += 2;
#if defined(LYRA_TIMING_DEBUG)
            static uint32_t logTimeCount = 0;
            static uint64_t logTimeTotalUs = 0;
            static uint32_t logTimeMaxUs = 0;
            static uint32_t nextLogReportMs = 0;

            logTimeCount++;
            logTimeTotalUs += logDurationUs;
            if (logDurationUs > logTimeMaxUs)
            {
                logTimeMaxUs = logDurationUs;
            }

            uint32_t logReportMs = millis();
            if (MP.sendserialon)
            {
                if ((int32_t)(logReportMs - nextLogReportMs) >= 0)
                {
                    Serial.printf(">loggingtime_avg_us: %f \n", (float)logTimeTotalUs / logTimeCount);
                    Serial.printf(">loggingtime_max_us: %lu \n", (unsigned long)logTimeMaxUs);
                    Serial.printf(">loggingtime_count: %lu \n", (unsigned long)logTimeCount);

                    logTimeCount = 0;
                    logTimeTotalUs = 0;
                    logTimeMaxUs = 0;
                    nextLogReportMs = logReportMs + TIMING_REPORT_PERIOD_MS;
                }
            }
#endif
        }
    }
    else
    {
        nextFlightLogUs = nowUs + CONTROL_PERIOD_US;
        if (millis() - MP.prevtime.logdata >= MP.intervals[currentState].logdata && NAV.newdata)
        {
            if (currentState == 0 || currentState == 5)
            {
                MP.logtobuf();
            }
            NAV.newdata = 0;
            MP.prevtime.logdata = millis();
            eventsfired += 2;
        }
    }


    if (millis()- MP.prevtime.led >= MP.intervals[currentState].led){
        MP.ledstate ? MP.setled(1) : MP.setled(0);
        MP.ledstate =! MP.ledstate;
        MP.prevtime.led = millis();
    }

    if (currentState == 0)
    {
        MP.beepcont();
    }
    else if (millis()- MP.prevtime.beep >= MP.intervals[currentState].beep){   
        if (!(MP._sysstate.r.uptime - brkout1statustime > 500))
        {
            MP.beep(5500, 100);
        }
        else
        {
            MP.beep(MP.freqs[currentState]);
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

    // if (MP._sysstate.r.uptime > 10000 && cameraon == false){
    //     P1.fire();
    // }

    MP.prevtime.loop = micros();
    MP._sysstate.r.state >= 1 ? MP.missionelasped = millis() - MP.liftofftime : MP.missionelasped = 0, MP.landedtime = millis();

    NAV.state = MP._sysstate.r.state;

    MP._sysstate.r.uptime = millis();
}
 

void loop1() { // nav core loop
    static uint32_t nextSampleUs = micros() + CONTROL_PERIOD_US;
#if defined(LYRA_NAV_PROFILING)
    static uint32_t navOverrunCount = 0;
#endif

    if (MP.usbMassStorageMode)
    {
        return;
    }

    uint32_t nowUs = micros();
    if (!scheduledTickDue(nowUs, nextSampleUs, CONTROL_PERIOD_US))
    {
        return;
    }

    uint8_t currentState = MP._sysstate.r.state;
    NAV.state = currentState;
    currentState == 0 ? NAV.useaccel = 1 : NAV.useaccel = 0;
    
#if defined(LYRA_NAV_PROFILING)
    uint32_t navLoopWorkStartUs = micros();
#endif

    NAV.prevtime.getdata = nowUs;
    NAV_PROFILE_BEGIN(getSensorProfileStartUs);
    NAV.getsensordata(MP.sendtoteleplot);
    NAV_PROFILE_END(NAV_PROF_GETSENSORDATA, getSensorProfileStartUs);
    if (MP.sendserialon && MP.sendtoteleplot)
    {
            //Serial.printf(">sensordatatime: %f \n",float(micros()-NAV.prevtime.getdata)/1000);
            NAV.prevtime.predictkf = micros();
    }
    NAV_PROFILE_BEGIN(kfProfileStartUs);
    NAV.KFrun();
    NAV_PROFILE_END(NAV_PROF_KFRUN, kfProfileStartUs);
    //Serial.printf(">kfpredicttime: %f \n",float(micros()-NAV.prevtime.predictkf)/1000);

    if (micros() - NAV.prevtime.kfupdate >= 100000)
    {
        NAV.prevtime.updatekf = micros();
        if (NAV.useaccel == 1)
    {
        NAV_PROFILE_BEGIN(accelAdjustProfileStartUs);
        NAV._sysstate.r.orientationquat = NAV.adjustwithaccel(0.1);
        NAV_PROFILE_END(NAV_PROF_ACCEL_ADJUST, accelAdjustProfileStartUs);
    }
        NAV.prevtime.kfupdate = micros();
        //Serial.printf(">kfupdatetime: %f \n",float(micros()-NAV.prevtime.updatekf)/1000);
    }

    NAV.newdata = 1;
#if defined(LYRA_NAV_PROFILING)
    uint32_t navLoopWorkUs = micros() - navLoopWorkStartUs;
    NAV_PROFILE_ADD(NAV_PROF_LOOP_WORK, navLoopWorkUs);
    if (navLoopWorkUs > CONTROL_PERIOD_US)
    {
        navOverrunCount++;
    }
#endif
#if defined(LYRA_TIMING_DEBUG)
    static uint32_t navCycleUs[NAV_RATE_AVG_SAMPLES] = {};
    static uint8_t navCycleIndex = 0;
    static uint8_t navCycleCount = 0;
    static uint64_t navCycleTotalUs = 0;
    static uint32_t nextRateReportMs = 0;

    uint32_t loopEndUs = micros();
    if (NAV.prevtime.looptime != 0)
    {
        uint32_t cycleUs = loopEndUs - NAV.prevtime.looptime;

        if (navCycleCount < NAV_RATE_AVG_SAMPLES)
        {
            navCycleCount++;
        }
        else
        {
            navCycleTotalUs -= navCycleUs[navCycleIndex];
        }

        navCycleUs[navCycleIndex] = cycleUs;
        navCycleTotalUs += cycleUs;
        navCycleIndex = (navCycleIndex + 1) % NAV_RATE_AVG_SAMPLES;
    }

    uint32_t reportMs = millis();
    if (MP.sendserialon && navCycleCount > 0 && (int32_t)(reportMs - nextRateReportMs) >= 0)
    {
        float averageRateHz = (1000000.0f * navCycleCount) / (float)navCycleTotalUs;
        Serial.printf(">navlooprate: %f \n", averageRateHz);

        nextRateReportMs = reportMs + TIMING_REPORT_PERIOD_MS;
    }
    NAV.prevtime.looptime = loopEndUs;
#else
    NAV.prevtime.looptime = micros();
#endif
#if defined(LYRA_NAV_PROFILING)
    static uint32_t nextProfileReportMs = 0;
    uint32_t profileReportMs = millis();
    if (MP.sendserialon && (int32_t)(profileReportMs - nextProfileReportMs) >= 0)
    {
        Serial.printf(">prof_period_us: %lu \n", (unsigned long)CONTROL_PERIOD_US);
        Serial.printf(">prof_overruns: %lu \n", (unsigned long)navOverrunCount);

        for (uint8_t i = 0; i < NAV_PROF_COUNT; i++)
        {
            const NavProfileBucketStats &bucket = NAV_PROFILE.buckets[i];
            if (bucket.count == 0)
            {
                continue;
            }

            Serial.printf(">prof_%s_avg_us: %f \n", NAV_PROFILE_NAMES[i], (float)bucket.totalUs / bucket.count);
            Serial.printf(">prof_%s_max_us: %lu \n", NAV_PROFILE_NAMES[i], (unsigned long)bucket.maxUs);
        }

        NAV_PROFILE.reset();
        navOverrunCount = 0;
        nextProfileReportMs = profileReportMs + TIMING_REPORT_PERIOD_MS;
    }
#endif
    NAV._sysstate.r.uptime = millis();
}

void tud_mount_cb(void)
{
}

void tud_umount_cb(void)
{
}
