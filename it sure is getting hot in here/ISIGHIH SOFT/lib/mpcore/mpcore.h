#if !defined(MPCOREHEADER)
#define MPCOREHEADER

#include <navcore.h>
//#include <pyrobatt.h>
#include <RPi_Pico_TimerInterrupt.h>

//fs::File logtofile;

NAVCORE NAV;
SERIALPORT port;

// PYROCHANNEL P1(1);
// PYROCHANNEL P2(2);
// PYROCHANNEL P3(3);
// PYROCHANNEL P4(4);

RPI_PICO_Timer PyroTimer0(0);

CircularBuffer<logpacket,LOGBUFSIZE> logbuf = {};

bool checkfirepyros(struct repeating_timer *t)
{ 
  (void) t;

    // P1.checkfire();
    // P2.checkfire();
    // P3.checkfire();
    // P4.checkfire();

    //Serial.println("checking pyros");

  return true;
}





class MPCORE{
    
    int buf[FLASH_PAGE_SIZE/sizeof(int)];  // One page buffer of ints
    int addr;
    int32_t first_empty_page = -1;

    struct CsvExportStreamState
    {
        uint32_t nextDataSector = 0;
        uint32_t cachedDataSector = 0xFFFFFFFF;
        uint32_t headerPos = 0;
        uint32_t page = 0;
        uint32_t lineLen = 0;
        uint32_t linePos = 0;
        char line[1024];
        uint8_t sector[512];
    };

    CsvExportStreamState exportStream;

    uint32_t exportFatSectors = 0;
    uint32_t exportRootDirSectors = 0;
    uint32_t exportDataStart = 0;
    uint32_t exportFileSectors = 0;
    uint32_t exportFileClusters = 0;
    uint32_t exportClusterCount = 0;
    uint32_t exportDataSectors = 0;
    uint32_t exportVolumeSerial = 0x49534947;
    uint8_t exportSectorsPerCluster = 1;

    void resetCsvExportStream();
    bool nextCsvExportByte(char *out);
    void buildNextCsvExportDataSector();
    int readCsvExportDataSector(uint32_t dataSector, uint8_t *sector);


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
        int readCsvExportSector(uint32_t lba, uint8_t *sector);

        int movebuftofile();

        int erasedata();
        int flashinit();
        int dumpdata();
        int previewdata();

        int changestate();
        int parsecommand(char input);
        int checkforpyros();

        float readbattvoltage();
        int calibrateimus();
        

};

MPCORE::MPCORE(){
    _sysstate.r.state = 0;
    _sysstate.r.errorflag = 0;
};

void MPCORE::setuppins(){
    pinMode(LEDPIN,OUTPUT);
    pinMode(BUZZERPIN,OUTPUT);
    pinMode(BATT_SENSE,INPUT);

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

// void MPCORE::beepcont(){
//     if (_sysstate.r.pyroscont & 0b1)
//     {
//         beep(4500);
//     }
//     else
//     {
//         beep(4000);
//     }
//     delay(50);
//         if (_sysstate.r.pyroscont & 0b10)
//     {
//         beep(4500);
//     }
//     else
//     {
//         beep(4000);
//     }
//     delay(50);
//         if (_sysstate.r.pyroscont & 0b100)
//     {
//         beep(4500);
//     }
//     else
//     {
//         beep(4000);
//     }
//     delay(50);
//         if (_sysstate.r.pyroscont & 0b1000)
//     {
//         beep(4500);
//     }
//     else
//     {
//         beep(4000);
//     }

// }

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

    // P1.timeout = 500;
    // P2.timeout = 500;
    // P3.timeout = 500;
    // P4.timeout = 500;

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
    Serial.print("MOVEDONE\n");
    //logfile.close();

    
    return 0;
}

int MPCORE::logdata(mpstate state, navpacket navstate){
    if (first_empty_page < 0 || first_empty_page >= (int32_t)(FLASH_LOG_SIZE / FLASH_PAGE_SIZE))
    {
        Serial.println("Flash log full, skipping entry");
        return 1;
    }

    //Serial.println("Writing");
    rp2040.idleOtherCore();
    //Serial.printf("\n>MPspot: %d \n", 0);
    NAV.event ? state.r.status = state.r.status | 0b1 : state.r.status = state.r.status;
    uint32_t openingtime = micros();
    state.r.batterystate = readbattvoltage();
 
    logpacket datatolog = preplogentry(state,navstate);
    //Serial.printf("\n>MPspot: %d \n", 1);
    //delay(50);
    datatolog.r.MPstate.r.missiontime = datatolog.r.MPstate.r.uptime - liftofftime;
    uint8_t logbuf[FLASH_PAGE_SIZE] = {};

    
    int j = 0;
    for (int i = 0; i < sizeof(datatolog.data); i++)
    {
        logbuf[j] = datatolog.data[j];
        j++;
    }
    //Serial.printf("\n>MPspot: %d \n", 2);
    //delay(50);
    //Serial.printf("\n>MPspot: %d \n", 3);
    //delay(50);
    //Serial.println("Writing to page #" + String(first_empty_page, DEC));
    
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (uint8_t *)logbuf, FLASH_PAGE_SIZE);
    restore_interrupts (ints);
    rp2040.resumeOtherCore();
    //Serial.printf("\n>MPspot: %d \n", 4);
    //delay(50);
    first_empty_page++;
    //Serial.print("DONEWRITING\n");
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
        Serial.printf("%f,%f,%f,",readentry.r.navsysstate.r.imudata.gyro.x*(180/M_PI),readentry.r.navsysstate.r.imudata.gyro.y*(180/M_PI), readentry.r.navsysstate.r.imudata.gyro.z*(180/M_PI));
        Serial.printf("%f,%f,",readentry.r.navsysstate.r.imudata.temp);
        Serial.printf("%f,",readentry.r.navsysstate.r.imudata.absaccel);
        Serial.printf("%d,%d,",readentry.r.MPstate.r.status,readentry.r.MPstate.r.MET);
        Serial.printf("%d,%f,%d\n",readentry.r.MPstate.r.state, readentry.r.MPstate.r.batterystate, 202);


        entrynum++;
        preventry = readentry;
    }
        
    Serial.println("done");
    return 0;
}

static const char CSV_EXPORT_HEADER[] =
    "index,checksum1,"
    "mp_errorflag,mp_uptime,mp_MET,mp_state,mp_status,mp_missiontime,mp_batterystate,"
    "nav_errorflag,nav_uptime,"
    "imu_accel_x,imu_accel_y,imu_accel_z,imu_gyro_x,imu_gyro_y,imu_gyro_z,imu_absaccel,imu_temp,"
    "rtd_temp1,rtd_temp2,"
    "checksum2\n";

static constexpr uint32_t CSV_EXPORT_LINE_SIZE = 320;

static int formatCsvExportLine(char *line, size_t lineSize, uint32_t index, const logpacket &packet)
{
    return snprintf(line, lineSize,
        "%lu,%u,"
        "%u,%lu,%lu,%u,%u,%ld,%f,"
        "%u,%lu,"
        "%f,%f,%f,%f,%f,%f,%f,%f,"
        "%f,%f,"
        "%u\n",
        (unsigned long)index,
        (unsigned int)packet.r.checksum1,
        (unsigned int)packet.r.MPstate.r.errorflag,
        (unsigned long)packet.r.MPstate.r.uptime,
        (unsigned long)packet.r.MPstate.r.MET,
        (unsigned int)packet.r.MPstate.r.state,
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
        packet.r.navsysstate.r.temp1,
        packet.r.navsysstate.r.temp2,
        (unsigned int)packet.r.checksum2);
}

static bool formatCsvExportFixedLine(char *line, size_t lineSize, uint32_t index, const logpacket &packet)
{
    if (lineSize < CSV_EXPORT_LINE_SIZE + 1)
    {
        return false;
    }

    int len = formatCsvExportLine(line, lineSize, index, packet);
    if (len < 0 || len >= (int)lineSize)
    {
        return false;
    }

    uint32_t payloadLen = (uint32_t)len;
    if (payloadLen > 0 && line[payloadLen - 1] == '\n')
    {
        payloadLen--;
    }

    if (payloadLen + 1 > CSV_EXPORT_LINE_SIZE)
    {
        return false;
    }

    memset(line + payloadLen, ' ', CSV_EXPORT_LINE_SIZE - payloadLen - 1);
    line[CSV_EXPORT_LINE_SIZE - 1] = '\n';
    line[CSV_EXPORT_LINE_SIZE] = '\0';
    return true;
}

void MPCORE::resetCsvExportStream(){
    exportStream.nextDataSector = 0;
    exportStream.cachedDataSector = 0xFFFFFFFF;
    exportStream.headerPos = 0;
    exportStream.page = 0;
    exportStream.lineLen = 0;
    exportStream.linePos = 0;
    memset(exportStream.sector, 0, sizeof(exportStream.sector));
}

bool MPCORE::nextCsvExportByte(char *out){
    const uint32_t headerLen = sizeof(CSV_EXPORT_HEADER) - 1;

    if (exportStream.headerPos < headerLen)
    {
        *out = CSV_EXPORT_HEADER[exportStream.headerPos++];
        return true;
    }

    while (exportStream.linePos >= exportStream.lineLen)
    {
        if (exportStream.page >= (uint32_t)first_empty_page)
        {
            return false;
        }

        logpacket packet = readdata(exportStream.page,false);
        int len = formatCsvExportLine(exportStream.line, sizeof(exportStream.line), exportStream.page, packet);
        exportStream.page++;

        if (len < 0 || len >= (int)sizeof(exportStream.line))
        {
            exportStream.lineLen = 0;
            exportStream.linePos = 0;
            return false;
        }

        exportStream.lineLen = (uint32_t)len;
        exportStream.linePos = 0;
    }

    *out = exportStream.line[exportStream.linePos++];
    return true;
}

void MPCORE::buildNextCsvExportDataSector(){
    memset(exportStream.sector, 0, sizeof(exportStream.sector));

    for (uint32_t i = 0; i < 512; i++)
    {
        char value;
        if (!nextCsvExportByte(&value))
        {
            break;
        }
        exportStream.sector[i] = (uint8_t)value;
    }

    exportStream.cachedDataSector = exportStream.nextDataSector;
    exportStream.nextDataSector++;
}

int MPCORE::readCsvExportDataSector(uint32_t dataSector, uint8_t *sector){
    if (dataSector >= exportDataSectors)
    {
        memset(sector, 0, 512);
        return 0;
    }

    memset(sector, 0, 512);

    uint64_t fileOffset = (uint64_t)dataSector * 512;
    if (fileOffset >= exportFileSize)
    {
        return 0;
    }

    const uint32_t headerLen = sizeof(CSV_EXPORT_HEADER) - 1;
    char line[CSV_EXPORT_LINE_SIZE + 1];
    uint32_t cachedPage = 0xFFFFFFFF;

    for (uint32_t i = 0; i < 512 && fileOffset + i < exportFileSize; i++)
    {
        uint64_t currentOffset = fileOffset + i;

        if (currentOffset < headerLen)
        {
            sector[i] = (uint8_t)CSV_EXPORT_HEADER[currentOffset];
            continue;
        }

        uint64_t rowOffset = currentOffset - headerLen;
        uint32_t page = rowOffset / CSV_EXPORT_LINE_SIZE;
        uint32_t lineOffset = rowOffset % CSV_EXPORT_LINE_SIZE;

        if (page >= (uint32_t)first_empty_page)
        {
            break;
        }

        if (cachedPage != page)
        {
            logpacket packet = readdata(page,false);
            if (!formatCsvExportFixedLine(line, sizeof(line), page, packet))
            {
                return 1;
            }
            cachedPage = page;
        }

        sector[i] = (uint8_t)line[lineOffset];
    }

    return 0;
}

int MPCORE::readCsvExportSector(uint32_t lba, uint8_t *sector){
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

    if (lba >= exportSectorCount)
    {
        memset(sector, 0, 512);
        return 0;
    }

    memset(sector, 0, 512);

    if (lba == 0)
    {
        sector[0] = 0xEB;
        sector[1] = 0x3C;
        sector[2] = 0x90;
        memcpy(sector + 3, "MSDOS5.0", 8);
        put16(sector + 11, 512);
        sector[13] = exportSectorsPerCluster;
        put16(sector + 14, 1);
        sector[16] = 2;
        put16(sector + 17, 512);
        put16(sector + 19, exportSectorCount <= 0xFFFF ? exportSectorCount : 0);
        sector[21] = 0xF8;
        put16(sector + 22, exportFatSectors);
        put16(sector + 24, 63);
        put16(sector + 26, 255);
        put32(sector + 28, 0);
        put32(sector + 32, exportSectorCount > 0xFFFF ? exportSectorCount : 0);
        sector[36] = 0x80;
        sector[38] = 0x29;
        put32(sector + 39, exportVolumeSerial);
        memcpy(sector + 43, "ISIGHIH CSV", 11);
        memcpy(sector + 54, "FAT16   ", 8);
        sector[510] = 0x55;
        sector[511] = 0xAA;
        return 0;
    }

    if (lba >= 1 && lba < 1 + 2 * exportFatSectors)
    {
        uint32_t fatSector = (lba - 1) % exportFatSectors;
        uint32_t fatEntries = exportClusterCount + 2;

        for (uint32_t i = 0; i < 256; i++)
        {
            uint32_t entry = fatSector * 256 + i;
            uint16_t value = 0;

            if (entry == 0)
            {
                value = 0xFFF8;
            }
            else if (entry == 1)
            {
                value = 0xFFFF;
            }
            else if (entry < fatEntries)
            {
                uint32_t firstFileCluster = 2;
                uint32_t lastFileCluster = firstFileCluster + exportFileClusters - 1;

                if (entry >= firstFileCluster && entry < lastFileCluster)
                {
                    value = (uint16_t)(entry + 1);
                }
                else if (entry == lastFileCluster)
                {
                    value = 0xFFFF;
                }
            }

            put16(sector + i * 2, value);
        }
        return 0;
    }

    uint32_t rootStart = 1 + 2 * exportFatSectors;
    if (lba >= rootStart && lba < rootStart + exportRootDirSectors)
    {
        if (lba == rootStart)
        {
            memcpy(sector + 0, "ISIGHIH ", 8);
            memcpy(sector + 8, "CSV", 3);
            sector[11] = 0x20;
            put16(sector + 26, 2);
            put32(sector + 28, exportFileSize);
        }
        return 0;
    }

    if (lba >= exportDataStart)
    {
        return readCsvExportDataSector(lba - exportDataStart, sector);
    }

    return 0;
}

int MPCORE::buildCsvExportFlash(){
    if (first_empty_page <= 0)
    {
        Serial.println("No flash data to export");
        return 1;
    }

    const uint32_t headerLen = sizeof(CSV_EXPORT_HEADER) - 1;
    static char line[CSV_EXPORT_LINE_SIZE + 1];

    for (uint32_t page = 0; page < first_empty_page; ++page)
    {
        logpacket packet = readdata(page,false);
        if (!formatCsvExportFixedLine(line, sizeof(line), page, packet))
        {
            Serial.println("CSV format error");
            return 1;
        }
    }

    exportFileSize = headerLen + ((uint32_t)first_empty_page * CSV_EXPORT_LINE_SIZE);
    exportFileSectors = (exportFileSize + 511) / 512;

    const uint8_t clusterSizes[] = {1, 2, 4, 8, 16, 32, 64};
    exportSectorsPerCluster = 0;
    for (uint8_t i = 0; i < sizeof(clusterSizes) / sizeof(clusterSizes[0]); i++)
    {
        uint8_t sectorsPerCluster = clusterSizes[i];
        uint32_t fileClusters = (exportFileSectors + sectorsPerCluster - 1) / sectorsPerCluster;
        uint32_t clusterCount = fileClusters < 8192 ? 8192 : fileClusters;

        if (clusterCount < 65525)
        {
            exportSectorsPerCluster = sectorsPerCluster;
            exportFileClusters = fileClusters;
            exportClusterCount = clusterCount;
            break;
        }
    }

    if (exportSectorsPerCluster == 0)
    {
        Serial.println("CSV export too large for FAT16 dynamic image");
        return 1;
    }

    exportDataSectors = exportClusterCount * exportSectorsPerCluster;
    exportRootDirSectors = ((512 * 32) + 511) / 512;
    exportFatSectors = ((exportClusterCount + 2) * 2 + 511) / 512;
    exportDataStart = 1 + 2 * exportFatSectors + exportRootDirSectors;
    exportSectorCount = exportDataStart + exportDataSectors;
    exportVolumeSerial = 0x49530000 ^ exportFileSize ^ ((uint32_t)first_empty_page << 16) ^ millis();

    resetCsvExportStream();
    Serial.printf("MSC dynamic export ready: %u bytes, %u sectors, %u sectors/cluster\n", exportFileSize, exportSectorCount, exportSectorsPerCluster);
    return 0;
}

int MPCORE::changestate(){

    Vector3d accelvec = vectorfloatto3(NAV._sysstate.r.imudata.accel);
    Vector3d gyrovec  = vectorfloatto3(NAV._sysstate.r.imudata.gyro);
    float accelmag = accelvec.norm();
    if (_sysstate.r.state == 0) // detect liftoff
    {
        
        NAV._sysstate.r.imudata.absaccel > 50 && _sysstate.r.uptime > 500 ? detectiontime = detectiontime : detectiontime = millis();
        if (millis() - detectiontime >= 400)
        {
            _sysstate.r.state = 1;
            detectiontime = millis();
            Serial.println("liftoff");
            liftofftime = millis();
            movebuftofile();
            Serial.println("liftedoff");
        }
        
    }

    else if (_sysstate.r.state == 4) // detect landing
    {   

        if (accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
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

    if (_sysstate.r.state >= 1 && _sysstate.r.state != 5 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 30)
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



// // 0 = ground idle  1 = powered ascent 2 = unpowered ascent 3 = ballisitic decsent 4 = under chute 5 = landed
// int MPCORE::checkforpyros(){

//     if (NAV._sysstate.r.filtered.alt < NAV._sysstate.r.filtered.maxalt && _sysstate.r.state >= 3)
//     {
//         _sysstate.r.pyrosfired =  _sysstate.r.pyrosfired | 0b11;
//         //Serial.println("deploying apogee");
//         P3.fire();

//     }  

//     if (NAV._sysstate.r.filtered.alt < 200 && _sysstate.r.state >= 3)
//     {
        
//         _sysstate.r.pyrosfired = _sysstate.r.pyrosfired | 0b10;
//         //Serial.println("deploying main");
//         P2.fire();
//     } 

//     int stagingstate = 0;
//     if (_sysstate.r.state == 2){
//         stagingstate = 1;
//     if (NAV._sysstate.r.orientationeuler.x*(180/M_PI) > 90-20){
//         stagingstate = 2;
//     if (NAV._sysstate.r.orientationeuler.x*(180/M_PI) < 90+20){
//         stagingstate = 3;
//     if(abs(NAV._sysstate.r.orientationeuler.y*(180/M_PI)) > 180-20){
//         stagingstate = 4;
//     if(NAV._sysstate.r.filtered.vvel > 20){
//         stagingstate = 5;
//     if(NAV._sysstate.r.filtered.alt > 100){
//         stagingstate = 6;
//     if(millis() - burnouttime > 1000)
//     {
//         stagingstate = 7;
//         _sysstate.r.pyrosfired = _sysstate.r.pyrosfired | 0b100;
//         P4.fire();
//     }
//     }}}}}}
//     else
//     {
//         //_sysstate.r.pyrosfired = _sysstate.r.pyrosfired & 0b11;
//     }
//     //Serial.printf(">stagingstate: %d\n",stagingstate);
    

//     P1.checkfire();
//     P2.checkfire();
//     P3.checkfire();
//     P4.checkfire();

//     uint8_t pyrocont = 0;
//     pyrocont = pyrocont | P1.getcont();
//     pyrocont = pyrocont | P2.getcont()*2;
//     pyrocont = pyrocont | P3.getcont()*4;
//     pyrocont = pyrocont | P4.getcont()*8;
//     _sysstate.r.pyroscont = pyrocont;

//     uint8_t pyrostate = 0;
//     pyrostate = pyrostate | P1.state();
//     pyrostate = pyrostate | P2.state()*2;
//     pyrostate = pyrostate | P3.state()*4;
//     pyrostate = pyrostate | P4.state()*8;
//     _sysstate.r.pyrostate = pyrostate;
//     return 0;
// }


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
    if (input == 'L'){
        Serial.println("put into landed mode");
        _sysstate.r.state = 5;
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
        //NAV.getpadoffset();
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
        // case '1':

        //     P1.fire();
        //     break;
        
        // case '2':
        //     P2.fire();
        //     break;
        
        // case '3':
        //     P3.fire();
        //     break;
        
        // case '4':
        //     P4.fire();
        //     break;
        
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

    // case 'i':
    // NAV.getcalibrationdata();
    // break;

    // case 'y':
    // NAV.dumpoffsets();
    // break;

    // case 'c':
    // calibrateimus();
    // break;

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
    return 0;
}



#endif // MPCOREHEADER


