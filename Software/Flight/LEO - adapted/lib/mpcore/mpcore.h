#if !defined(MPCOREHEADER)
#define MPCOREHEADER

#include <navcore.h>
#include <pyrobatt.h>
#include <RPi_Pico_TimerInterrupt.h>

//fs::File logtofile;

NAVCORE NAV;
SERIALPORT port;

PYROCHANNEL P1(1);
PYROCHANNEL P2(2);
PYROCHANNEL P3(3);
PYROCHANNEL P4(4);

RPI_PICO_Timer PyroTimer0(0);

CircularBuffer<logpacket,LOGBUFSIZE> logbuf = {};

bool checkfirepyros(struct repeating_timer *t)
{ 
  (void) t;

    P1.checkfire();
    P2.checkfire();
    P3.checkfire();
    P4.checkfire();

    //Serial.println("checking pyros");

  return true;
}





class MPCORE{
    
    int buf[FLASH_PAGE_SIZE/sizeof(int)];  // One page buffer of ints
    int addr;
    int32_t first_empty_page = -1;


    public:
        
        int radiook = 1;
        mpstate _sysstate;
        int detectiontime = 0;
        uint32_t landedtime = 0;
        bool datamoved = false;
        uint32_t landingdetectiontime = 0;
        uint32_t liftofftime = 0;
        uint32_t missionelasped = 0;
        uint32_t burnouttime = 0;
        bool ready = false;
        bool beepon = 1;
        bool dogpspassthrough = false;
        bool usbMassStorageMode = false;
        bool exportReady = false;
        uint32_t exportFileSize = 0;
        uint32_t exportSectorCount = 0;
        bool flushpending = false;
        




        MPCORE();

        //int32_t _sysstate.errorflag = 0;
        /*
            0 = no error
            1 = serial init failure
            10 = radio init fail
        */
        bool sendserialon = false;
        bool sendtoteleplot = true;
        
        int freqs[6] = {3000,5000,5000,5000,5000,8000};


        struct timings{
            uint32_t logdata;
            uint32_t led;
            uint32_t serial;
            uint32_t sendtelemetry;
            uint32_t beep;
            uint32_t loop;
        };
        timings intervals[6] = {
            {17,1000,200,1000,10000}, // ground idle
            {17,500,200, 200,800}, // powered ascent
            {17,500,200,333,800}, // unpowered ascent
            {17,500,200,333,800}, // ballistic descent
            {17,800,200,333,800}, //ready to land
            {1000,1500,200,1500,500} // landed
        };
        timings prevtime;
        bool ledstate = false;



        void setuppins();
        int initperipherials();

        void beep();
        void beep(int freq);
        void beep(int freq, unsigned int duration);
        void beepcont();

        void setled(int state);
        
        int flashtest();
        int logdata(mpstate state,navpacket navstate);
        logpacket readdata(uint64_t page,bool serial);
        int logtobuf();
        int buildCsvExportFlash();
        int flushbufferpage();

        int movebuftofile();

        int erasedata();
        int flashinit();
        int dumpdata();
        int previewdata();

        int changestate();
        int parsecommand(char input);
        int sendtelemetry();
        int checkforpyros();

        float readbattvoltage();
        int calibrateimus();
        

};

MPCORE::MPCORE(){
    _sysstate.r.state = 0;
    _sysstate.r.errorflag = 0;
    _sysstate.r.pyrosfired = 0b000;
};

void MPCORE::setuppins(){
    pinMode(LEDPIN,OUTPUT);
    pinMode(BUZZERPIN,OUTPUT);
    pinMode(BATT_SENSE,INPUT);
    pinMode(BRKOUT1,INPUT);

    digitalWrite(LEDPIN, HIGH);
    
    return;
}

void MPCORE::setled(int state){
    digitalWrite(LEDPIN,state);
    return;
}

void MPCORE::beep(){
    if (beepon){
        tone(BUZZERPIN,4000,200);
        delay(200);
        noTone(BUZZERPIN);
    }
    

    return;
}

void MPCORE::beep(int freq){
    if (beepon){
        tone(BUZZERPIN,freq,200);
        delay(200);
        noTone(BUZZERPIN);
    }
    return;
}

void MPCORE::beep(int freq, unsigned int duration){
    if (beepon)
    {
        tone(BUZZERPIN,freq,duration);
        delay(duration);
        noTone(BUZZERPIN);
    }
return;
}

void MPCORE::beepcont(){
    if (_sysstate.r.pyroscont & 0b1)
    {
        beep(4500);
    }
    else
    {
        beep(4000);
    }
    delay(50);
        if (_sysstate.r.pyroscont & 0b10)
    {
        beep(4500);
    }
    else
    {
        beep(4000);
    }
    delay(50);
        if (_sysstate.r.pyroscont & 0b100)
    {
        beep(4500);
    }
    else
    {
        beep(4000);
    }
    delay(50);
        if (_sysstate.r.pyroscont & 0b1000)
    {
        beep(4500);
    }
    else
    {
        beep(4000);
    }

}

float MPCORE::readbattvoltage(){
    long raw = analogRead(BATT_SENSE);
    long adjusted  = map(raw,0,1230,0,1023);
    double calculated = double(adjusted)/100;
    return calculated;
}

int MPCORE::flashtest(){
    // Serial.println("FLASH_PAGE_SIZE = " + String(FLASH_PAGE_SIZE, DEC));
    // Serial.println("FLASH_SECTOR_SIZE = " + String(FLASH_SECTOR_SIZE,DEC));
    // Serial.println("FLASH_BLOCK_SIZE = " + String(FLASH_BLOCK_SIZE, DEC));
    // Serial.println("PICO_FLASH_SIZE_BYTES = " + String(PICO_FLASH_SIZE_BYTES, DEC));
    // Serial.println("XIP_BASE = 0x" + String(XIP_BASE, HEX));
    
    // Read the flash using memory-mapped addresses
    // For that we must skip over the XIP_BASE worth of RAM
    // int addr = FLASH_TARGET_OFFSET + XIP_BASE;
    int32_t page = 0;
    int *p;
    while (page < FLASH_LOG_SIZE/FLASH_PAGE_SIZE)
    {
        addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
        p = (int *)addr;
        // Serial.print("First four bytes of page " + String(page, DEC) );
        // Serial.print("( at 0x" + (String(int(p), HEX)) + ") = ");
        // Serial.println(*p);
        if( *p == -1 && first_empty_page < 0){
            first_empty_page = page;
            Serial.printf(" empty page at %d", +first_empty_page);
            return 0;
        }
        page++;
    }

    first_empty_page = FLASH_LOG_SIZE / FLASH_PAGE_SIZE;
    Serial.println("Flash log region full");

    
    // *buf = 123456;

    // if (first_empty_page < 0){
    //     Serial.println("Full sector, erasing...");
    //     uint32_t ints = save_and_disable_interrupts();
    //     flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    //     first_empty_page = 0;
    //     restore_interrupts (ints);
    // }
    // Serial.println("Writing to page #" + String(first_empty_page, DEC));
    // rp2040.idleOtherCore();
    // uint32_t ints = save_and_disable_interrupts();
    // flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (uint8_t *)buf, FLASH_PAGE_SIZE);
    // restore_interrupts (ints);
    // rp2040.resumeOtherCore();
    // Serial.println("out of test");
    return 0;
}



int MPCORE::initperipherials(){
    int error = port.init();
    error ? _sysstate.r.errorflag || 0b1 : _sysstate.r.errorflag;
    error ? _sysstate.r.errorflag || 0b10 : _sysstate.r.errorflag;
    _sysstate.r.batterystate = readbattvoltage();
    

    if (PyroTimer0.attachInterruptInterval(100 * 1000, checkfirepyros))
    {
      //Serial.print(F("Starting ITimer0 OK, millis() = ")); Serial.println(millis());
    }
    else
        Serial.println(F("Can't set ITimer0. Select another freq. or timer"));

    flashtest();

    P1.timeout = 500;
    P2.timeout = 500;
    P3.timeout = 500;
    P4.timeout = 500;

    return 0;
}


int MPCORE::logtobuf(){

    NAV.event ? _sysstate.r.status = _sysstate.r.status | 0b1 : _sysstate.r.status = _sysstate.r.status;
    uint32_t openingtime = micros();
    _sysstate.r.batterystate = readbattvoltage();

    logpacket datatolog = preplogentry(_sysstate,NAV._sysstate);
    logbuf.unshift(datatolog);

    return 0;
}

int MPCORE::movebuftofile(){
    Serial.print("BUF!");
    //fs::File logfile = LittleFS.open("/log.csv", "a+");
    for (int i = 0; i < LOGBUFSIZE; i++)
    {
        if (logbuf.isEmpty())
        {
            Serial.print("BUF EMPTY!");
            return 0;
        }
        
        logpacket datatolog = logbuf.pop();
        datatolog.r.MPstate.r.missiontime = datatolog.r.MPstate.r.uptime - liftofftime;
        logdata(datatolog.r.MPstate,datatolog.r.navsysstate);
    }
    
    //logfile.close();

    
    return 0;
}

int MPCORE::flushbufferpage(){
    if (logbuf.isEmpty())
    {
        return 0;
    }

    logpacket datatolog = logbuf.pop();
    datatolog.r.MPstate.r.missiontime = datatolog.r.MPstate.r.uptime - liftofftime;
    logdata(datatolog.r.MPstate, datatolog.r.navsysstate);
    return 1;
}

int MPCORE::logdata(mpstate state, navpacket navstate){

    if (first_empty_page < 0 || first_empty_page >= (int32_t)(FLASH_LOG_SIZE / FLASH_PAGE_SIZE))
    {
        Serial.println("Flash log full, skipping entry");
        return 1;
    }

    NAV.event ? state.r.status = state.r.status | 0b1 : state.r.status = state.r.status;
    uint32_t openingtime = micros();
    state.r.batterystate = readbattvoltage();
 
    logpacket datatolog = preplogentry(state,navstate);

    datatolog.r.MPstate.r.missiontime = datatolog.r.MPstate.r.uptime - liftofftime;
    uint8_t logbuf[FLASH_PAGE_SIZE] = {};
    int j = 0;
    // Serial.println("datapacket is: ");
    // for (int i = 0; i < sizeof(logbuf); i++)
    // {
    //     Serial.printf("%x ",datatolog.data[j]);
    //     j++;
    // }
    // Serial.println("");
    // j = 0;
    for (int i = 0; i < sizeof(datatolog.data); i++)
    {
        logbuf[j] = datatolog.data[j];
        j++;
    }
    
    // Serial.println("packet is: ");
    // j = 0;   
    // for (int i = 0; i < sizeof(logbuf); i++)
    // {
    //     Serial.printf("%x ",logbuf[j]);
    //     j++;
    // }
    // Serial.println("");

    //Serial.println("Writing to page #" + String(first_empty_page, DEC));
    rp2040.idleOtherCore();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (uint8_t *)logbuf, FLASH_PAGE_SIZE);
    restore_interrupts (ints);
    rp2040.resumeOtherCore();
    
    first_empty_page++;
    
    return 0;
}

logpacket MPCORE::readdata(uint64_t pageadr,bool serial){
    uint8_t *p;
    addr = XIP_BASE + FLASH_TARGET_OFFSET + (pageadr * FLASH_PAGE_SIZE);
    p = (uint8_t *)addr;
    // Serial.printf("Firstbytes of page %d",pageadr);
    // Serial.printf("( at 0x %d ) = ",int(*p));
    // Serial.println(*p);
    int j = 0;
    logpacket readpacket;
    for (int i = 0; i < sizeof(readpacket.data); i++)
    {
        readpacket.data[j] = *(uint8_t *)(addr+j);
        j++;
    }
    // Serial.println("read packet is: ");
    // j = 0;
    // for (int i = 0; i < sizeof(readpacket.data); i++)
    // {
    //     Serial.printf("%x ",readpacket.data[j]);
    //     j++;
    // }
    
    serial ? port.senddata(readpacket.r.MPstate,readpacket.r.navsysstate) : 1 == 1;
    return readpacket;
}

int MPCORE::erasedata(){
    Serial.println("erasing...");
    rp2040.idleOtherCore();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_LOG_SIZE);
    first_empty_page = 0;
    restore_interrupts (ints);
    rp2040.resumeOtherCore();
    return 0;
}

int MPCORE::previewdata(){

    uint32_t entrynum = 0;
    uint32_t page = 0;
    while (page < first_empty_page)
    {
        readdata(page,true);
        page++;
    }

    Serial.println("done");
    return 0;
}

int MPCORE::dumpdata(){
    Serial.println("dumping data to serial");
    Serial.println("newfile");
    Serial.println("index, checksum,uptime mp,uptime nav,missiontime,  errorflag mp,errorflag NAV,  accel x, accel y, accel z, accelworld x, accelworld y, accelworld z, mag x, mag y, mag z, accelhighg x, accelhighg y, accelhighg z, gyro x, gyro y, gyro z, euler x, euler y, euler z, quat w, quat x, quat y, quat z,quatadj w, quatadj x, quatadj y, quatadj z, altitude, presusre, verticalvel,filtered vvel, maxalt, altitudeagl, filtered alt, imutemp, barotemp,abs imu accel,abs adxl accel, pad altitude, baro max alt, mag raw x, mag raw y, mag raw z, cov x, cov y, cov z, sats, status, met, filtered accel ,state,battstate,pyros fired,pyros cont,pyros state, checksum2");
    uint32_t entrynum = 0;
    logpacket preventry;
    while (entrynum < first_empty_page)
    {
        logpacket readentry = readdata(entrynum,false);
        if (readentry.r.MPstate.r.missiontime < preventry.r.MPstate.r.missiontime)
        {
            Serial.println("newfile");
            Serial.println("index, checksum,uptime mp,uptime nav,missiontime,  errorflag mp,errorflag NAV,  accel x, accel y, accel z, accelworld x, accelworld y, accelworld z, mag x, mag y, mag z, accelhighg x, accelhighg y, accelhighg z," 
            "gyro x, gyro y, gyro z, euler x, euler y, euler z, quat w, quat x, quat y, quat z,quatadj w, quatadj x, quatadj y, quatadj z, altitude, presusre, verticalvel,filtered vvel, maxalt, altitudeagl, filtered alt, imutemp," 
            "barotemp,abs imu accel,abs adxl accel, pad altitude, baro max alt, cov x, cov y, cov z, status, met, filtered accel ,state,battstate,pyros fired,pyros cont,pyros state, checksum2");
        }
        
        Serial.printf("%d, 101,",entrynum);
        Serial.printf("%d,%d,%d,",readentry.r.MPstate.r.uptime,readentry.r.navsysstate.r.uptime,readentry.r.MPstate.r.missiontime);
        Serial.printf("%d,%d,",readentry.r.MPstate.r.errorflag,readentry.r.navsysstate.r.errorflag);
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.imudata.accel.x,readentry.r.navsysstate.r.imudata.accel.y,readentry.r.navsysstate.r.imudata.accel.z);
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.accelworld.x,readentry.r.navsysstate.r.accelworld.y,readentry.r.navsysstate.r.accelworld.z);
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.magdata.utesla.x, readentry.r.navsysstate.r.magdata.utesla.y, readentry.r.navsysstate.r.magdata.utesla.z);
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.adxldata.accel.x, readentry.r.navsysstate.r.adxldata.accel.y, readentry.r.navsysstate.r.adxldata.accel.z);
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.imudata.gyro.x*(180/M_PI),readentry.r.navsysstate.r.imudata.gyro.y*(180/M_PI), readentry.r.navsysstate.r.imudata.gyro.z*(180/M_PI));
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.orientationeuler.x*(180/M_PI), readentry.r.navsysstate.r.orientationeuler.y*(180/M_PI), readentry.r.navsysstate.r.orientationeuler.z*(180/M_PI));
        Serial.printf("%f,%f,%f,%f,",readentry.r.navsysstate.r.orientationquat.w, readentry.r.navsysstate.r.orientationquat.x,readentry.r.navsysstate.r.orientationquat.y, readentry.r.navsysstate.r.orientationquat.z);
        Serial.printf("%f,%f,%f,%f,",readentry.r.navsysstate.r.orientationquatadj.w, readentry.r.navsysstate.r.orientationquatadj.x,readentry.r.navsysstate.r.orientationquatadj.y, readentry.r.navsysstate.r.orientationquatadj.z);
        Serial.printf("%f,%f,",readentry.r.navsysstate.r.barodata.altitude, readentry.r.navsysstate.r.barodata.pressure);
        Serial.printf("%f,%f,",readentry.r.navsysstate.r.barodata.verticalvel, readentry.r.navsysstate.r.filtered.vvel);
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.barodata.maxrecordedalt, readentry.r.navsysstate.r.barodata.altitudeagl, readentry.r.navsysstate.r.filtered.alt);
        Serial.printf("%f,%f,",readentry.r.navsysstate.r.imudata.temp, readentry.r.navsysstate.r.barodata.temp);
        Serial.printf("%f,%f,",readentry.r.navsysstate.r.imudata.absaccel,readentry.r.navsysstate.r.adxldata.absaccel);
        Serial.printf("%f,%f,",readentry.r.navsysstate.r.barodata.padalt,readentry.r.navsysstate.r.barodata.maxrecordedalt);
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.covariences.x, readentry.r.navsysstate.r.covariences.y,readentry.r.navsysstate.r.covariences.z);
        Serial.printf("%d,%d,",readentry.r.MPstate.r.status,readentry.r.MPstate.r.MET);
        Serial.printf("%f,",readentry.r.navsysstate.r.filtered.vertaccel);
        Serial.printf("%d,%f,%d,%d,%d,%d\n",readentry.r.MPstate.r.state, readentry.r.MPstate.r.batterystate, readentry.r.MPstate.r.pyrosfired, readentry.r.MPstate.r.pyroscont, readentry.r.MPstate.r.pyrostate, 202);


        entrynum++;
        preventry = readentry;
    }
        
    Serial.println("done");
    return 0;
}

static const char CSV_EXPORT_HEADER[] =
    "index,checksum1,"
    "mp_errorflag,mp_uptime,mp_MET,mp_state,mp_pyrosfired,mp_pyroscont,mp_pyrostate,mp_status,mp_missiontime,mp_batterystate,"
    "nav_errorflag,nav_uptime,"
    "imu_accel_x,imu_accel_y,imu_accel_z,imu_gyro_x,imu_gyro_y,imu_gyro_z,imu_absaccel,imu_temp,"
    "baro_pressure,baro_altitude,baro_temp,baro_verticalvel,baro_maxrecordedalt,baro_altitudeagl,baro_padalt,"
    "adxl_accel_x,adxl_accel_y,adxl_accel_z,adxl_absaccel,"
    "mag_gauss_x,mag_gauss_y,mag_gauss_z,mag_utesla_x,mag_utesla_y,mag_utesla_z,"
    "accelworld_x,accelworld_y,accelworld_z,"
    "orientationeuler_x,orientationeuler_y,orientationeuler_z,"
    "orientationquat_w,orientationquat_x,orientationquat_y,orientationquat_z,"
    "orientationquatadj_w,orientationquatadj_x,orientationquatadj_y,orientationquatadj_z,"
    "filtered_vertaccel,filtered_alt,filtered_vvel,filtered_maxalt,"
    "covariences_x,covariences_y,covariences_z,"
    "checksum2\n";

static int formatCsvExportLine(char *line, size_t lineSize, uint32_t index, const logpacket &packet)
{
    return snprintf(line, lineSize,
        "%lu,%u,"
        "%u,%lu,%lu,%u,%u,%u,%u,%u,%ld,%f,"
        "%u,%lu,"
        "%f,%f,%f,%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,"
        "%f,%f,%f,%f,%f,%f,"
        "%f,%f,%f,"
        "%f,%f,%f,"
        "%f,%f,%f,%f,"
        "%f,%f,%f,%f,"
        "%f,%f,%f,%f,"
        "%f,%f,%f,"
        "%u\n",
        (unsigned long)index,
        (unsigned int)packet.r.checksum1,
        (unsigned int)packet.r.MPstate.r.errorflag,
        (unsigned long)packet.r.MPstate.r.uptime,
        (unsigned long)packet.r.MPstate.r.MET,
        (unsigned int)packet.r.MPstate.r.state,
        (unsigned int)packet.r.MPstate.r.pyrosfired,
        (unsigned int)packet.r.MPstate.r.pyroscont,
        (unsigned int)packet.r.MPstate.r.pyrostate,
        (unsigned int)packet.r.MPstate.r.status,
        (long)packet.r.MPstate.r.missiontime,
        packet.r.MPstate.r.batterystate,
        (unsigned int)packet.r.navsysstate.r.errorflag,
        (unsigned long)packet.r.navsysstate.r.uptime,
        packet.r.navsysstate.r.imudata.accel.x,
        packet.r.navsysstate.r.imudata.accel.y,
        packet.r.navsysstate.r.imudata.accel.z,
        packet.r.navsysstate.r.imudata.gyro.x,
        packet.r.navsysstate.r.imudata.gyro.y,
        packet.r.navsysstate.r.imudata.gyro.z,
        packet.r.navsysstate.r.imudata.absaccel,
        packet.r.navsysstate.r.imudata.temp,
        packet.r.navsysstate.r.barodata.pressure,
        packet.r.navsysstate.r.barodata.altitude,
        packet.r.navsysstate.r.barodata.temp,
        packet.r.navsysstate.r.barodata.verticalvel,
        packet.r.navsysstate.r.barodata.maxrecordedalt,
        packet.r.navsysstate.r.barodata.altitudeagl,
        packet.r.navsysstate.r.barodata.padalt,
        packet.r.navsysstate.r.adxldata.accel.x,
        packet.r.navsysstate.r.adxldata.accel.y,
        packet.r.navsysstate.r.adxldata.accel.z,
        packet.r.navsysstate.r.adxldata.absaccel,
        packet.r.navsysstate.r.magdata.gauss.x,
        packet.r.navsysstate.r.magdata.gauss.y,
        packet.r.navsysstate.r.magdata.gauss.z,
        packet.r.navsysstate.r.magdata.utesla.x,
        packet.r.navsysstate.r.magdata.utesla.y,
        packet.r.navsysstate.r.magdata.utesla.z,
        packet.r.navsysstate.r.accelworld.x,
        packet.r.navsysstate.r.accelworld.y,
        packet.r.navsysstate.r.accelworld.z,
        packet.r.navsysstate.r.orientationeuler.x,
        packet.r.navsysstate.r.orientationeuler.y,
        packet.r.navsysstate.r.orientationeuler.z,
        packet.r.navsysstate.r.orientationquat.w,
        packet.r.navsysstate.r.orientationquat.x,
        packet.r.navsysstate.r.orientationquat.y,
        packet.r.navsysstate.r.orientationquat.z,
        packet.r.navsysstate.r.orientationquatadj.w,
        packet.r.navsysstate.r.orientationquatadj.x,
        packet.r.navsysstate.r.orientationquatadj.y,
        packet.r.navsysstate.r.orientationquatadj.z,
        packet.r.navsysstate.r.filtered.vertaccel,
        packet.r.navsysstate.r.filtered.alt,
        packet.r.navsysstate.r.filtered.vvel,
        packet.r.navsysstate.r.filtered.maxalt,
        packet.r.navsysstate.r.covariences.x,
        packet.r.navsysstate.r.covariences.y,
        packet.r.navsysstate.r.covariences.z,
        (unsigned int)packet.r.checksum2);
}

int MPCORE::buildCsvExportFlash(){
    if (first_empty_page <= 0)
    {
        Serial.println("No flash data to export");
        return 1;
    }

    uint32_t fileSize = sizeof(CSV_EXPORT_HEADER) - 1;
    static char line[4096];

    for (uint32_t page = 0; page < first_empty_page; ++page)
    {
        logpacket packet = readdata(page,false);
        int len = formatCsvExportLine(line, sizeof(line), page, packet);

        if (len < 0 || len >= (int)sizeof(line))
        {
            Serial.println("CSV format error");
            return 1;
        }
        fileSize += (uint32_t)len;
    }

    uint32_t totalSectors = MSC_BLOCK_COUNT;
    uint32_t rootDirSectors = ((16 * 32) + 511) / 512;
    uint32_t fatSectors = 0;
    uint32_t clusterCount = 0;
    uint32_t prevFatSectors;

    do {
        prevFatSectors = fatSectors;
        uint32_t dataSectors = totalSectors - 1 - rootDirSectors - 2 * fatSectors;
        clusterCount = dataSectors;
        fatSectors = ((clusterCount + 2) * 2 + 511) / 512;
    } while (fatSectors != prevFatSectors);

    uint32_t dataStart = 1 + 2 * fatSectors + rootDirSectors;
    uint32_t fileClusters = (fileSize + 511) / 512;
    if (clusterCount < 4085 || clusterCount >= 65525)
    {
        Serial.println("MSC geometry is not FAT16-compatible");
        return 1;
    }
    if (fileClusters > clusterCount)
    {
        Serial.println("MSC export too large for reserved flash region");
        return 1;
    }

    static uint8_t sector[512];
    memset(sector, 0, sizeof(sector));

    // clear export region
    rp2040.idleOtherCore();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(MSC_FLASH_OFFSET, MSC_FLASH_SIZE);
    restore_interrupts(ints);
    rp2040.resumeOtherCore();

    // boot sector
    memset(sector, 0, sizeof(sector));
    auto put16 = [](uint8_t *dst, uint16_t value) {
        dst[0] = value & 0xFF;
        dst[1] = (value >> 8) & 0xFF;
    };
    auto put32 = [](uint8_t *dst, uint32_t value) {
        dst[0] = value & 0xFF;
        dst[1] = (value >> 8) & 0xFF;
        dst[2] = (value >> 16) & 0xFF;
        dst[3] = (value >> 24) & 0xFF;
    };
    sector[0] = 0xEB;
    sector[1] = 0x3C;
    sector[2] = 0x90;
    memcpy(sector + 3, "MSDOS5.0", 8);
    put16(sector + 11, 512);
    sector[13] = 1;
    put16(sector + 14, 1);
    sector[16] = 2;
    put16(sector + 17, 16);
    put16(sector + 19, totalSectors <= 0xFFFF ? totalSectors : 0);
    sector[21] = 0xF8;
    put16(sector + 22, fatSectors);
    put16(sector + 24, 63);
    put16(sector + 26, 255);
    put32(sector + 28, 0);
    put32(sector + 32, totalSectors > 0xFFFF ? totalSectors : 0);
    sector[36] = 0x80;
    sector[38] = 0x29;
    put32(sector + 39, 0x4C595241);
    memcpy(sector + 43, "LYRA CSV   ", 11);
    memcpy(sector + 54, "FAT16   ", 8);
    sector[510] = 0x55;
    sector[511] = 0xAA;

    rp2040.idleOtherCore();
    ints = save_and_disable_interrupts();
    flash_range_program(MSC_FLASH_OFFSET + 0, sector, 512);
    restore_interrupts(ints);
    rp2040.resumeOtherCore();

    // build FAT table
    uint32_t fatEntries = clusterCount + 2;
    uint16_t *fat = (uint16_t *)malloc(fatEntries * sizeof(uint16_t));
    if (!fat)
    {
        Serial.println("Out of memory for FAT table");
        return 1;
    }
    fat[0] = 0xFFF8;
    fat[1] = 0xFFFF;
    for (uint32_t c = 2; c < fatEntries; ++c)
    {
        if (c < 2 + fileClusters - 1)
        {
            fat[c] = (uint16_t)(c + 1);
        }
        else if (c == 2 + fileClusters - 1)
        {
            fat[c] = 0xFFFF;
        }
        else
        {
            fat[c] = 0x0000;
        }
    }

    memset(sector, 0, sizeof(sector));
    uint32_t fatOffset = MSC_FLASH_OFFSET + 512;
    uint32_t fatBytes = fatEntries * sizeof(uint16_t);
    for (uint32_t s = 0; s < fatSectors; ++s)
    {
        uint32_t copyLen = fatBytes - s * 512;
        if (copyLen > 512) copyLen = 512;
        memcpy(sector, ((uint8_t *)fat) + s * 512, copyLen);
        rp2040.idleOtherCore();
        ints = save_and_disable_interrupts();
        flash_range_program(fatOffset + s * 512, sector, 512);
        restore_interrupts(ints);
        rp2040.resumeOtherCore();
        memset(sector, 0, sizeof(sector));
    }

    for (uint32_t s = 0; s < fatSectors; ++s)
    {
        uint32_t copyLen = fatBytes - s * 512;
        if (copyLen > 512) copyLen = 512;
        memcpy(sector, ((uint8_t *)fat) + s * 512, copyLen);
        rp2040.idleOtherCore();
        ints = save_and_disable_interrupts();
        flash_range_program(fatOffset + (fatSectors + s) * 512, sector, 512);
        restore_interrupts(ints);
        rp2040.resumeOtherCore();
        memset(sector, 0, sizeof(sector));
    }
    free(fat);

    // root directory
    memset(sector, 0, sizeof(sector));
    memcpy(sector + 0, "FLIGHT  ", 8);
    memcpy(sector + 8, "CSV", 3);
    sector[11] = 0x20;
    put16(sector + 26, 2);
    put32(sector + 28, fileSize);

    uint32_t rootOffset = MSC_FLASH_OFFSET + 512 + 2 * fatSectors * 512;
    rp2040.idleOtherCore();
    ints = save_and_disable_interrupts();
    flash_range_program(rootOffset, sector, 512);
    restore_interrupts(ints);
    rp2040.resumeOtherCore();

    // write file data sequentially
    uint32_t dataStartOffset = MSC_FLASH_OFFSET + dataStart * 512;
    uint32_t currentSector = 0;
    uint32_t sectorPos = 0;
    memset(sector, 0, sizeof(sector));

    auto appendExportBytes = [&](const char *data, uint32_t len) {
        uint32_t written = 0;
        while (written < len)
        {
            uint32_t chunk = len - written;
            if (chunk > 512 - sectorPos) chunk = 512 - sectorPos;
            memcpy(sector + sectorPos, data + written, chunk);
            written += chunk;
            sectorPos += chunk;

            if (sectorPos == 512)
            {
                rp2040.idleOtherCore();
                ints = save_and_disable_interrupts();
                flash_range_program(dataStartOffset + currentSector * 512, sector, 512);
                restore_interrupts(ints);
                rp2040.resumeOtherCore();
                currentSector++;
                sectorPos = 0;
                memset(sector, 0, sizeof(sector));
            }
        }
    };

    appendExportBytes(CSV_EXPORT_HEADER, sizeof(CSV_EXPORT_HEADER) - 1);

    for (uint32_t page = 0; page < first_empty_page; ++page)
    {
        logpacket packet = readdata(page,false);
        int len = formatCsvExportLine(line, sizeof(line), page, packet);

        if (len < 0 || len >= (int)sizeof(line))
        {
            Serial.println("CSV format error while writing");
            return 1;
        }

        appendExportBytes(line, (uint32_t)len);
    }

    if (sectorPos > 0)
    {
        rp2040.idleOtherCore();
        ints = save_and_disable_interrupts();
        flash_range_program(dataStartOffset + currentSector * 512, sector, 512);
        restore_interrupts(ints);
        rp2040.resumeOtherCore();
        currentSector++;
    }

    exportFileSize = fileSize;
    exportSectorCount = totalSectors;
    Serial.printf("MSC export image built: %u bytes, %u sectors\n", fileSize, exportSectorCount);
    return 0;
}

int MPCORE::changestate(){

    Vector3d accelvec = vectorfloatto3(NAV._sysstate.r.imudata.accel);
    Vector3d gyrovec  = vectorfloatto3(NAV._sysstate.r.imudata.gyro);
    float accelmag = accelvec.norm();
    if (_sysstate.r.state == 0) // detect liftoff
    {
        
        NAV._sysstate.r.filtered.alt > 8 && _sysstate.r.uptime > 500 ? detectiontime = detectiontime : detectiontime = millis();
        if (millis() - detectiontime >= 400)
        {
            _sysstate.r.state = 1;
            detectiontime = millis();
            Serial.println("liftoff");
            liftofftime = millis();
            flushpending = true;
        }
        
    }

    else if (_sysstate.r.state == 1) // detect burnout
    {

        NAV._sysstate.r.accelworld.z < 8 ? detectiontime = detectiontime : detectiontime = millis();
        
        if (millis() - detectiontime >= 200)
        {
            _sysstate.r.state = 2;
            detectiontime = millis();
            Serial.println("burnout");
            burnouttime = millis();
        }
    }

    else if (_sysstate.r.state == 2) // detect appogee
    {
        NAV._sysstate.r.filtered.alt < NAV._sysstate.r.filtered.maxalt *0.99 && NAV._sysstate.r.filtered.vvel < 50 ?  detectiontime = detectiontime : detectiontime = millis();

        if (millis() - detectiontime >= 300)
        {
            _sysstate.r.state = 3;
            detectiontime = millis();
            Serial.println("appogee");
        }
    }

    else if (_sysstate.r.state == 3) // detect chute opening
    {
        

        accelmag > 7 ? detectiontime = detectiontime : detectiontime = millis();
        if (millis() - detectiontime >= 500)
        {
            _sysstate.r.state = 4;
            detectiontime = millis();
            Serial.println("chutes out");
        }
    }

    else if (_sysstate.r.state == 4) // detect landing
    {   

        if (abs(NAV._sysstate.r.barodata.verticalvel) < 0.3 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
        {
                detectiontime = detectiontime;
        }
        else{
            detectiontime = millis();
        }

        if (millis() - detectiontime >= 3000)
        {
            _sysstate.r.state = 5;
            detectiontime = millis();
            landedtime = millis();
            Serial.println("landed");
        }
    }

    if (_sysstate.r.state > 1 && _sysstate.r.state != 5 && abs(NAV._sysstate.r.barodata.verticalvel) < 0.3 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
    {
        landingdetectiontime = landingdetectiontime;
    }
    else{
        landingdetectiontime = millis();
    }
    if (millis() - landingdetectiontime >= 30000)
        {
            _sysstate.r.state = 5;
            detectiontime = millis();
            landedtime = millis();
            Serial.println("randoland");
    }

    return 0;
}



// 0 = ground idle  1 = powered ascent 2 = unpowered ascent 3 = ballisitic decsent 4 = under chute 5 = landed
int MPCORE::checkforpyros(){

    if (NAV._sysstate.r.filtered.alt < NAV._sysstate.r.filtered.maxalt && _sysstate.r.state >= 3)
    {
        _sysstate.r.pyrosfired =  _sysstate.r.pyrosfired | 0b11;
        //Serial.println("deploying apogee");
        P3.fire();

    }  

    if (NAV._sysstate.r.filtered.alt < 200 && _sysstate.r.state >= 3)
    {
        
        _sysstate.r.pyrosfired = _sysstate.r.pyrosfired | 0b10;
        //Serial.println("deploying main");
        P2.fire();
    } 

    int stagingstate = 0;
    if (_sysstate.r.state == 2){
        stagingstate = 1;
    if (NAV._sysstate.r.orientationeuler.x*(180/M_PI) > 90-20){
        stagingstate = 2;
    if (NAV._sysstate.r.orientationeuler.x*(180/M_PI) < 90+20){
        stagingstate = 3;
    if(abs(NAV._sysstate.r.orientationeuler.y*(180/M_PI)) > 180-20){
        stagingstate = 4;
    if(NAV._sysstate.r.filtered.vvel > 20){
        stagingstate = 5;
    if(NAV._sysstate.r.filtered.alt > 100){
        stagingstate = 6;
    if(millis() - burnouttime > 1000)
    {
        stagingstate = 7;
        _sysstate.r.pyrosfired = _sysstate.r.pyrosfired | 0b100;
        P4.fire();
    }
    }}}}}}
    else
    {
        //_sysstate.r.pyrosfired = _sysstate.r.pyrosfired & 0b11;
    }
    //Serial.printf(">stagingstate: %d\n",stagingstate);
    

    P1.checkfire();
    P2.checkfire();
    P3.checkfire();
    P4.checkfire();

    uint8_t pyrocont = 0;
    pyrocont = pyrocont | P1.getcont();
    pyrocont = pyrocont | P2.getcont()*2;
    pyrocont = pyrocont | P3.getcont()*4;
    pyrocont = pyrocont | P4.getcont()*8;
    _sysstate.r.pyroscont = pyrocont;

    uint8_t pyrostate = 0;
    pyrostate = pyrostate | P1.state();
    pyrostate = pyrostate | P2.state()*2;
    pyrostate = pyrostate | P3.state()*4;
    pyrostate = pyrostate | P4.state()*8;
    _sysstate.r.pyrostate = pyrostate;
    return 0;
}


int MPCORE::parsecommand(char input){
    if (int(input) == 0)
    {
        return 1;
    }
    Serial.println(int(input));
    int32_t page;
    char channel;
    logpacket currentpacket;
    int j = 0;

    if (input == 'l' && _sysstate.r.state < 1){
        Serial.println("put into launch mode");
            _sysstate.r.state = 1;
            detectiontime = millis();
            Serial.println("liftoff");
            liftofftime = millis();
            movebuftofile();
        return 0;
    }

    else if (input == 'a' && (_sysstate.r.state < 4 || _sysstate.r.state >= 6 )){
        _sysstate.r.state = 0;
        return 0;
    }

    switch (input)
    {
    case 's':
        Serial.println("printing data to teleplot");
        sendserialon = !sendserialon;
        NAV.sendtoserial = !NAV.sendtoserial;
        port.sendtoplot = true;
        break;

    case 'w':
        Serial.println("printing data to processing");
        sendserialon = !sendserialon;
        port.sendtoplot = false;
        break;

    case 'b':
        Serial.println("toggle beeping");
        beepon = !beepon;
        break;
    
    case 'X':
        Serial.println("getting new offsets for adxl");
        adxl.getnewoffsets();
        break;

    case 'B':
        Serial.println("dumping databuf");
        j = 0;
        for (int i = 0; i < LOGBUFSIZE; i++)
        {
            currentpacket = logbuf[j];
            port.senddata(currentpacket.r.MPstate,currentpacket.r.navsysstate);
            j++;
        }
        break;

    case 'D':

        dumpdata();
        break;
    
    case 't':
        Serial.println(" hitltesting");
        NAV.hitltime = millis();
        NAV.hitlteston = 1;
        break;
    case 'r':
        page = Serial.parseInt();
        Serial.println("testing");
        readdata(page,true);
        break;
    
    case 'd':
        previewdata();
        break;
    
    case 'o':
        NAV.getpadoffset();
        break;
    
    case 'P':
        if (Serial.read() != 'A')
        {
            return 0;
        }
        channel = Serial.read();
        Serial.print("firing pyro ");
        Serial.println(channel);
        switch (channel)
        {
        case '1':

            P1.fire();
            break;
        
        case '2':
            P2.fire();
            break;
        
        case '3':
            P3.fire();
            break;
        
        case '4':
            P4.fire();
            break;
        
        default:
            break;
        }
        break;
    
    case 'e':
        Serial.println("erasing flash?");
        if (Serial.read() == 'e')
        {
            Serial.println("erasing flash");
            uint32_t erasestarttime = millis();
            uint32_t messagetime = millis();
            while (millis()-erasestarttime < 10000)
            {
                if (millis() - messagetime > 500)
                {
                    Serial.println("ERASING FLASH ------ ERASING FLASH ------ ERASING FLASH ------ TO ABORT ERASION DISCONNECT POWER OR TYPE A CHARACTER");
                    messagetime = millis();
                }
                if (Serial.available())
                {
                    return 1;
                }
                
            }
            
            erasedata();
        }
        break;

    case 'i':
    NAV.getcalibrationdata();
    break;

    case 'y':
    NAV.dumpoffsets();
    break;

    case 'c':
    calibrateimus();
    break;

    case 'g':
    dogpspassthrough = !dogpspassthrough;
    break;

    case 'M':
        Serial.println("MSC export mode requested");
        usbMassStorageMode = true;
        exportReady = false;
        break;
    
    default:
        break;
    }
    
    
    return 0;
}


int MPCORE::calibrateimus(){
    imu.bcal = 0.5*(imu.calibrationpos+imu.calibrationneg);
    adxl.bcal = 0.5*(adxl.calibrationpos+imu.calibrationneg);
    return 0;
}



#endif // MPCOREHEADER


