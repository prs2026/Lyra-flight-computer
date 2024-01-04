#if !defined(MPCOREHEADER)
#define MPCOREHEADER

#include <navcore.h>
#include <pyrobatt.h>

//fs::File logtofile;

NAVCORE NAV;
SERIALPORT port;
RADIO telemetryradio;

class MPCORE{
    

    public:
        

        mpstate _sysstate;
        int detectiontime = 0;
        uint32_t landedtime = 0;
        bool datamoved = false;
        uint32_t landingdetectiontime = 0;
        uint32_t liftofftime = 0;
        uint32_t missionelasped = 0;



        MPCORE();

        //int32_t _sysstate.errorflag = 1;
        /*
            1 = no error
            3 = handshake fail
            5 = serial init failure
            7 = sd init fail
            11 = flash init fail
            13 = no packet recived
            17 = bad telemetry packet
            19 = radio init fail
            23 = bad telemetry packet
        */
        bool sendserialon = false;
        bool sendtoteleplot = true;

        int ledcolor;


        struct timings{
            uint32_t logdata;
            uint32_t led;
            uint32_t serial;
            uint32_t sendtelemetry;
            uint32_t beep;
            uint32_t detectstatechange;
            uint32_t loop;
        };
        timings intervals[7] = {
            {2000,1000,50,1000,30000,10}, // ground idle
            {10,200,100, 200, 500,10}, // launch detect
            {10,500,100, 200, 1000,10}, // powered ascent
            {10,500,100,200, 1000,10}, // unpowered ascent
            {10,500,100,200, 1000,10}, // ballistic descent
            {10,800,100,200, 1000,10}, //ready to land
            {1000,1500,100,200, 1000,10} // landed
        };
        timings prevtime;
        bool ledstate = false;



        void setuppins();
        int initperipherials();

        void beep();
        void beep(int freq);
        void beep(int freq, unsigned int duration);

        void setled(int color);

        int logtextentry(const char *entry);
        int logtextentry(const char *entry, float val);
        int logtextentry(const char *entry, int val);
        int logcurrentstate();

        int logdata();
        int erasedata();
        int flashinit();

        int changestate();
        int parsecommand(char input);
        int sendtelemetry();

};

MPCORE::MPCORE(){
    _sysstate.r.state = 0;
    _sysstate.r.errorflag = 1;
};

void MPCORE::setuppins(){
    pinMode(LEDRED,OUTPUT);
    pinMode(LEDGREEN,OUTPUT);
    pinMode(LEDBLUE,OUTPUT);
    pinMode(BUZZERPIN,OUTPUT);


    digitalWrite(LEDRED, LOW);
    digitalWrite(LEDGREEN, HIGH);
    digitalWrite(LEDBLUE, HIGH);

    // adc.setuppins();
    return;
}


void MPCORE::beep(){
    tone(BUZZERPIN,2000,200);
    return;
}

void MPCORE::beep(int freq){
    tone(BUZZERPIN,freq,200);
    return;
}

void MPCORE::beep(int freq, unsigned int duration){
    tone(BUZZERPIN,freq,duration);
return;
}


void MPCORE::setled(int color){
    switch (color)
    {
    case OFF:
        digitalWrite(LEDRED, HIGH);
        digitalWrite(LEDGREEN, HIGH);
        digitalWrite(LEDBLUE, HIGH);
        break;

    case RED:
        digitalWrite(LEDRED, LOW);
        digitalWrite(LEDGREEN, HIGH);
        digitalWrite(LEDBLUE, HIGH);
        break;
    
    case GREEN:
        digitalWrite(LEDRED, HIGH);
        digitalWrite(LEDGREEN, LOW);
        digitalWrite(LEDBLUE, HIGH);
        break;

    case BLUE:
        digitalWrite(LEDRED, HIGH);
        digitalWrite(LEDGREEN, HIGH);
        digitalWrite(LEDBLUE, LOW);
        break;
    
    default:
        break;
    }
}

int MPCORE::logtextentry(const char *entry){
    fs::File textfile = LittleFS.open("/textlog.txt", "a+");
    textfile.printf("| %d | %d | ", millis(),missionelasped);
    textfile.print(entry);
    textfile.println();
    Serial.printf("| %d | %d | ", millis(),missionelasped);
    Serial.print(entry);
    Serial.println();

    textfile.close();
    return 0;
}

int MPCORE::logtextentry(const char *entry, float val){
    fs::File textfile = LittleFS.open("/textlog.txt", "a+");
    textfile.printf("| %d | %d | ", millis(),missionelasped);
    textfile.print(entry);
    textfile.print(val);
    textfile.println();

    textfile.close();
    return 0;
}

int MPCORE::logtextentry(const char *entry, int val){
    fs::File textfile = LittleFS.open("/textlog.txt", "a+");
    
    textfile.printf("| %d | %d | ", millis(),missionelasped);
    textfile.print(entry);
    textfile.print(val);
    textfile.println();

    textfile.close();
    return 0;
}

int MPCORE::logcurrentstate(){
    fs::File textfile = LittleFS.open("/textlog.txt", "a+");
    textfile.printf("| %d | %d | ", millis(),0);
    textfile.println("setup complete, status: ");
    textfile.printf("\tMP errorflag: %d \n\tNAV errorflag: %d ",_sysstate.r.errorflag);


    textfile.close();

    return 0;
}


int MPCORE::initperipherials(){
    port.init();
    int error = telemetryradio.init();
    // adc.setuppins();
    
    return 0;
}



int MPCORE::logdata(){
    uint32_t openingtime = micros();
    // adc.readbatt();
    // _sysstate.r.batterystate = adc.battvoltage;

    logpacket datatolog = preplogentry(_sysstate,NAV._sysstate);
    //Serial.print(datatolog.r.checksum2);

    fs::File logfile = LittleFS.open("/log.csv", "a+");
    //Serial.printf("opening file took %d \n",micros()-openingtime);
    openingtime = micros();
    if (!logfile){
        return 1;
        _sysstate.r.errorflag % 11 == 0 ? _sysstate.r.errorflag = _sysstate.r.errorflag : _sysstate.r.errorflag *= 11;
    };
    //Serial.printf("checking file took %d \n",micros()-openingtime);
    openingtime = micros();
    int j = 0;
    for (int i = 0; i < sizeof(logpacket); i++)
    {
        logfile.write(datatolog.data[j]);
        j++;
    }

    
    openingtime = micros();
    logfile.close();
    //Serial.printf("closing file took %d \n\n",micros()-openingtime);
    return 0;
}

int MPCORE::erasedata(){
    int error = LittleFS.remove("/log.csv");
    if (error != 1)
    {
        Serial.println("file erase fail");
        return 1;
    }
    Serial.println("file erase success");
    error = LittleFS.remove("/textlog.txt");
    if (error != 1)
    {
        Serial.println("log erase fail");
        return 1;
    }
    Serial.println("log erase success");
    return 0;
    
}


int MPCORE::flashinit(){
        Serial.println("flash init start");
        // LittleFSConfig cfg;
        // cfg.setAutoFormat(false);
        // LittleFS.setConfig(cfg);

        //rp2040.fifo.idleOtherCore();
        //delay(200);
        Serial.println("other core idle, trying to begin littlefs");
        int error = LittleFS.begin();

        if (error = 0)
        {
            Serial.printf("filesystem mount fail %d\n",error);
            _sysstate.r.errorflag *= 11;
            rp2040.resumeOtherCore();
            return 1;
        }
        //Serial.println("littlefs started!");

        // error = LittleFS.format();

        // if (error != 0)
        // {
        //     Serial.printf("filesystem format fail %d\n", error);
        //     errorflag *= 11;
        //     rp2040.resumeOtherCore();
        //     return 1;
        // }

        FSInfo64 *info;
        error = LittleFS.info64(*info);

        if (error != 1)
        {
            Serial.printf("filesystem info fail %d\n", error);
            _sysstate.r.errorflag *= 11;
            rp2040.resumeOtherCore();
            return 1;
        }
        

        uint32_t total = info->totalBytes;
        uint32_t used = info->usedBytes;
        uint32_t avail = total - used;

        Serial.printf("FS info: total %d, used %d, avail %d\n",total,used,avail);

        LittleFS.remove("/f.txt");

        fs::File testfile = LittleFS.open("/f.txt","w+");

        if (!testfile)
        {
            Serial.println("file open failed");
            _sysstate.r.errorflag *= 11;
            rp2040.resumeOtherCore();
            return 2;
        }

        //Serial.println("file opened");
        int testnum = 1;

        testfile.print(testnum);
        //Serial.print("file written");
        testfile.close();
        testfile = LittleFS.open("/f.txt","r");

        int readnum = testfile.read() - 48;

        //Serial.println("file read");

        if (readnum != testnum)
        {
            Serial.printf("read fail, expected %d, got %d\n",testnum,readnum);
            _sysstate.r.errorflag *= 11;
            rp2040.resumeOtherCore();
            return 3;
        }
        
        //Serial.printf("read success, expected %d, got %d\n",testnum,readnum);

        
        testfile.close();
        Serial.println("flash init complete");
        rp2040.resumeOtherCore();
        return 0;
}




int MPCORE::changestate(){

    Vector3d accelvec = vectorfloatto3(NAV._sysstate.r.imudata.accel);
    Vector3d gyrovec  = vectorfloatto3(NAV._sysstate.r.imudata.gyro);
    if (_sysstate.r.state == 1) // detect liftoff
    {
        
        float accelmag = accelvec.norm();
        accelmag > 20 ? detectiontime = detectiontime : detectiontime = millis();
        if (millis() - detectiontime >= 50)
        {
            _sysstate.r.state = 2;
            detectiontime = millis();
            logtextentry("liftoff detected");
            ebyte.setPower(Power_27,true);
        }
        
    }
    else if (_sysstate.r.state == 2) // detect burnout
    {
        
        float accelmag = accelvec.norm();
        accelvec.z() < 2 ? detectiontime = detectiontime : detectiontime = millis();
        if (millis() - detectiontime >= 200)
        {
            _sysstate.r.state = 3;
            detectiontime = millis();
            logtextentry("burnout detected");
        }
    }

    else if (_sysstate.r.state == 3) // detect appogee
    {
        NAV._sysstate.r.barodata.altitudeagl < NAV._sysstate.r.barodata.maxrecordedalt*0.95 ?  detectiontime = detectiontime : detectiontime = millis();

        if (millis() - detectiontime >= 100)
        {
            _sysstate.r.state = 4;
            detectiontime = millis();
            logtextentry("apogee detected");
        }
    }

    else if (_sysstate.r.state == 4) // detect chute opening
    {
        
        float accelmag = accelvec.norm();
        accelmag > 7 ? detectiontime = detectiontime : detectiontime = millis();
        if (millis() - detectiontime >= 300)
        {
            _sysstate.r.state = 5;
            detectiontime = millis();
            logtextentry("chutes opening detected");
        }
    }

    else if (_sysstate.r.state == 5) // detect landing
    {   

        

        if (abs(NAV._sysstate.r.barodata.verticalvel) < 0.3 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
        {
                detectiontime = detectiontime;
        }
        else{
            detectiontime = millis();
        }

        if (millis() - detectiontime >= 500)
        {
            _sysstate.r.state = 6;
            detectiontime = millis();
            landedtime = millis();
            logtextentry("landing detected");
        }
    }
    

    if (_sysstate.r.state > 1 && abs(NAV._sysstate.r.barodata.verticalvel) < 0.3 && accelvec.norm() < 20 &&  accelvec.norm() > 5  && gyrovec.norm() < 0.5)
    {
        landingdetectiontime = landingdetectiontime;
    }
    else{
        landingdetectiontime = millis();
    }
    if (millis() - landingdetectiontime >= 1000)
        {
            _sysstate.r.state = 6;
            detectiontime = millis();
            landedtime = millis();
            logtextentry("landing detected");
    }
    

    

    

    return 0;
}

int MPCORE::parsecommand(char input){
    if (int(input) == 0)
    {
        return 1;
    }
    Serial.println(int(input));
    
    

    if (input == 'l' && _sysstate.r.state < 2){
        _sysstate.r.state = 1;
        logtextentry("put into launch detect state");
        return 0;
    }

    else if (input == 'a' && (_sysstate.r.state < 3 || _sysstate.r.state >= 6 )){
        _sysstate.r.state = 0;
        ebyte.setPower(Power_21,true);
        logtextentry("aborted from state: ",int(_sysstate.r.state));
        return 0;
    }

    switch (input)
    {
    case 's':
        Serial.println("printing data to teleplot");
        sendserialon = !sendserialon;
        port.sendtoplot = true;
        break;

    case 'w':
        Serial.println("printing data to processing");
        sendserialon = !sendserialon;
        port.sendtoplot = false;
        break;

    case 'D':
        logdata();
        break;
    
    case 'o':
        NAV.getpadoffset();
        logtextentry("got new pad offset: ",float(NAV._sysstate.r.barodata.padalt));
        break;
    
    case 'k':
        Serial.println("testing radio");
        ebyte.setRadioMode(MODE_NORMAL);
        Serial1.print(0x32);
        break;

    
    default:
        break;
    }
    
    
    return 0;
}

int MPCORE::sendtelemetry(){
    telepacket packettosend;
    uint8_t databufs[32];

    packettosend = statetopacket(_sysstate,NAV._sysstate);
    telemetryradio.sendpacket(packettosend);

    
    return 0;
    
}



#endif // MPCOREHEADER


