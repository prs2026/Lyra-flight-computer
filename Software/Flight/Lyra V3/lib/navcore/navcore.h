#if !defined(NAVCOREHEADER)
#define NAVCOREHEADER

#include <Lyrav2sensors.h>

IMU imu;
BARO baro;

//using Eigen::MatrixXd;
//using Eigen::Vector3d;



class NAVCORE{

    const float ALTVAR 0.5 
    const float VVELVAR 0.8
    const float VACCELVAR 0.6
    const float ORIENTVAR 0.4

    const float ALTNOISE 0.1
    const float VVELNOISE 1.5
    const float VACCELNOISE 0.2
    const float ORIENTNOISE 0.4

    
    navpacket prevsysstate;
    uint32_t kfupdatetime;
    uint32_t kfpredicttime;

    public:
    
        navpacket _sysstate;

        
        

        float alpha = 0.98;

        void KFinit();

        NAVCORE();;
        /*
        1 = no errors
        3 = failed handshake
        5 = i2c devices fail
        7 = accel init fail
        11 = gyro init fail
        13 = baro init fail
        17 = mag init fail
        19 = packet send fail
        negative = fatal error
        */
        struct timings{
            uint32_t sendpacket;
            uint32_t intergrateorientation;
            uint32_t kfupdate;
            uint32_t looptime;
            uint32_t getdata;
            uint32_t predictkf;
            uint32_t updatekf;
        };
        timings intervals[6] = {
            {50}, // ground idle
            {50}, // powered ascent
            {50}, // unpowered ascent
            {50}, // ballistic descent
            {50}, //ready to land
            {50} // landed
        }; 
        timings prevtime;


        int initi2c();

        uint32_t sensorinit();
        void getsensordata();

        void KFpredict();                                                         
        void KFupdate();

        void intergrategyros(double timestep);

        void adjustwithaccel();

        navpacket computeorientation(navpacket currentpacket);
        Vector3float getworldaccel(navpacket _state);

        

};

#endif // NAVCOREHEADER