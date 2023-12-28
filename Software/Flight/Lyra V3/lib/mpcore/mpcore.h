#if !defined(MPCOREHEADER)
#define MPCOREHEADER


#include <navcore.h>
#include <pyrobatt.h>


//fs::File logtofile;


SERIALPORT port;
RADIO telemetryradio;

NAVCORE NAV;
ADC adc;


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
        void beep(int state,bool statewise);

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
#endif // MPCOREHEADER


