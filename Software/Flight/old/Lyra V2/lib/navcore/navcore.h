#if !defined(NAVCOREHEADER)
#define NAVCOREHEADER

#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "Lyrav2sensors.h"
#include "macros.h"
#include "SPI.h"
#include "SD.h"
#include <string.h>
#include "LittleFS.h"
//#include <ArduinoEigenDense.h>
#include <generallib.h>

IMU imu;
BARO baro;
MAG mag;

//using Eigen::MatrixXd;
//using Eigen::Vector3d;


class NAVCORE{
    
    navpacket prevsysstate;
    uint32_t kfupdatetime;
    uint32_t kfpredicttime;

    public:
    
        navpacket _sysstate;

        
        

        float alpha = 0.98;

        void KFinit(){
            _sysstate.r.filtered.alt = _sysstate.r.barodata.altitudeagl;
            _sysstate.r.filtered.vvel = _sysstate.r.barodata.verticalvel;
            _sysstate.r.filtered.accel = _sysstate.r.accelworld;
           
            _sysstate.r.uncertainty.alt = 100;
            _sysstate.r.uncertainty.vvel = 100;
            _sysstate.r.uncertainty.vaccel = 100;
            prevsysstate = _sysstate;
            prevtime.kfupdate = micros();
        }

        NAVCORE(){
            _sysstate.r.orientationquat = {1,0,0,0};
            _sysstate.r.errorflag = 1;
        };
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
        timings intervals[7] = {
            {50}, // ground idle
            {50}, // launch detect
            {50}, // powered ascent
            {50}, // unpowered ascent
            {50}, // ballistic descent
            {50}, //ready to land
            {50} // landed
        }; 
        timings prevtime;


        // int sendpacket(navpacket datatosend){
        //     if (rp2040.fifo.available() > 250)
        //     {
        //         return 1;
        //     }
            
        //     rp2040.fifo.push(0xAB);
        //     for (int i = 0; i < sizeof(datatosend.data)/sizeof(datatosend.data[0]); i++)
        //     {
        //         bool error = 1;
        //         int j = 0;
        //         rp2040.fifo.push(datatosend.data[i]);
                
        //     }
        //     rp2040.fifo.push(0xCD);
        //     return 0;
        // }

        // int handshake(){
        //     uint32_t data;
        //     rp2040.fifo.push(0xAA);
        //     data = rp2040.fifo.pop();
        //     if (data != 0xAB)
        //     {
        //         rp2040.fifo.push(data);
        //         _sysstate.r.errorflag*3;
        //         return 1;
        //     }
        //     rp2040.fifo.push(0xCD);
        //     rp2040.fifo.pop();
        //     navpacket handshakepacket;
        //     handshakepacket.r.errorflag = 1;

        //     return 0;
        // }

        int initi2c(){
            Wire1.setSCL(SCL);
            Wire1.setSDA(SDA);
            Wire1.setClock(10000);
            Wire1.begin();
            scani2c(true) ? _sysstate.r.errorflag*= 5 : _sysstate.r.errorflag *= 1;
            return 0;
        }

        uint32_t sensorinit(){
            int imustatus;
            int barostatus;
            int magstatus;
            imustatus = imu.init();;
            imustatus == 1 ? _sysstate.r.errorflag *= 7 : _sysstate.r.errorflag *= 1;
            imustatus == 2 ? _sysstate.r.errorflag *= 11 : _sysstate.r.errorflag *= 1;
            barostatus = baro.init();
            barostatus ? _sysstate.r.errorflag *= 13 : _sysstate.r.errorflag *= 1;
            magstatus = mag.init();
            magstatus ? _sysstate.r.errorflag *= 17 : _sysstate.r.errorflag *= 1;

            if (magstatus != 0 || imustatus != 0 || barostatus != 0)
            {
                _sysstate.r.errorflag *= -1;
            }
            
            
            return 0;
        }

        void getsensordata(){
            #if defined(VERBOSETIMES)
            uint32_t timestep = micros();
            imu.read();
            Serial.printf(">imureadtime: %f \n", float(micros()-timestep)/1000);
            timestep = micros();
            baro.readsensor();
            Serial.printf(">baroreadtime: %f \n", float(micros()-timestep)/1000);
            timestep = micros();
            mag.read();
            Serial.printf(">magreadtime: %f \n", float(micros()-timestep)/1000);
            #endif // VERBOSETIMES
            
            #if !defined(VERBOSETIMES)
            imu.read();
            baro.readsensor();
            mag.read();
            #endif // VERBOSETIMES


            _sysstate.r.magdata = mag.data;
            _sysstate.r.imudata = imu.data;
            _sysstate.r.barodata = baro.data;
            return;
        }

        void KFpredict(){
            navpacket extrapolatedsysstate = _sysstate;

            double timestep = (micros() - kfpredicttime)/1e6;

            extrapolatedsysstate.r.filtered.alt = _sysstate.r.filtered.alt + (timestep*_sysstate.r.filtered.vvel); // extrapolate with velocity dynamics
            extrapolatedsysstate.r.filtered.vvel = _sysstate.r.filtered.vvel + (timestep*_sysstate.r.accelworld.y); 


            extrapolatedsysstate.r.uncertainty.alt = _sysstate.r.uncertainty.alt + ((timestep*timestep)*_sysstate.r.uncertainty.vvel) + ALTVAR; // extrapolate variences with velocity dynamics
            extrapolatedsysstate.r.uncertainty.vvel = _sysstate.r.uncertainty.vvel + VVELVAR;



            //intergrategyros(timestep);
            extrapolatedsysstate = computeorientation(extrapolatedsysstate);
            extrapolatedsysstate.r.accelworld = getworldaccel(extrapolatedsysstate);
            extrapolatedsysstate.r.orientationeuler = quat2euler(extrapolatedsysstate.r.orientationquat);

            //Serial.printf(">extrap var: %f\n",extrapolatedsysstate.r.confidence.alt);

            kfpredicttime = micros();
            _sysstate = extrapolatedsysstate;
        }                                                         



        void KFupdate(){
            double timestep = (micros() - kfupdatetime)/1e6;

            variences kgain; // calc new kalman gain
            kgain.alt = _sysstate.r.uncertainty.alt/(_sysstate.r.uncertainty.alt+ALTNOISE);
            kgain.vvel = _sysstate.r.uncertainty.vvel/(_sysstate.r.uncertainty.vvel+VVELNOISE);
            //Serial.printf(">kalman gain: %f\n",kgain.alt);
            
            _sysstate.r.filtered.alt = prevsysstate.r.filtered.alt + kgain.alt*(_sysstate.r.barodata.altitudeagl - prevsysstate.r.filtered.alt); // state update
            _sysstate.r.filtered.vvel = prevsysstate.r.filtered.vvel + kgain.vvel*(_sysstate.r.barodata.verticalvel - prevsysstate.r.filtered.vvel);

            

            _sysstate.r.uncertainty.alt = (1-kgain.alt)*prevsysstate.r.uncertainty.alt; // variences update
            _sysstate.r.uncertainty.vvel = (1-kgain.vvel)*prevsysstate.r.uncertainty.vvel;
            
            //adjustwithaccel();

            prevsysstate = _sysstate;
            kfupdatetime = micros();
        }

        void intergrategyros(double timestep){
            Quaterniond orientationquat = quatstructtoeigen(_sysstate.r.orientationquat);
            Vector3d gyro = vectorfloatto3(_sysstate.r.imudata.gyro);
            
            Quaterniond qdelta(AngleAxisd(timestep*gyro.norm(), gyro.normalized()));

            orientationquat = orientationquat * qdelta;

            _sysstate.r.orientationquat = eigentoquatstruct(orientationquat);

            return;
        }


        void adjustwithaccel(){
            Quaterniond orientationquat = quatstructtoeigen(_sysstate.r.orientationquat);
            Vector3d accel = vectorfloatto3(_sysstate.r.imudata.accel);

            Quaterniond accelquat;

            accelquat.x() = accel.x();
            accelquat.y() = accel.y();
            accelquat.z() = accel.z();

            accelquat = orientationquat * accelquat * orientationquat.inverse();

            accel.x() = accelquat.x();
            accel.y() = accelquat.y();
            accel.z() = accelquat.z();
            

            Vector3d accelnorm(accel.normalized());

            float phi = acos(accelnorm.y());

            Vector3d naxis(accelnorm.cross(Vector3d(0,1,0)));

            naxis = naxis.normalized();


            Quaterniond accelrotquat(AngleAxisd((1-alpha)*phi,naxis));

            orientationquat = accelrotquat * orientationquat;

        }


        navpacket computeorientation(navpacket currentpacket){
            double timestep = (micros() - prevtime.intergrateorientation)/1e6;
            Quaterniond orientationquat = quatstructtoeigen(currentpacket.r.orientationquat);
            Vector3d gyro = vectorfloatto3(currentpacket.r.imudata.gyro);
            Vector3d accel = vectorfloatto3(currentpacket.r.imudata.accel);
            Vector3d orientationeuler = vectorfloatto3(currentpacket.r.orientationeuler);
            
            Quaterniond qdelta(AngleAxisd(timestep*gyro.norm(), gyro.normalized()));

            orientationquat = orientationquat * qdelta;

            prevtime.intergrateorientation = micros();

            Quaterniond accelquat;

            accelquat.x() = accel.x();
            accelquat.y() = accel.y();
            accelquat.z() = accel.z();

            accelquat = orientationquat * accelquat * orientationquat.inverse();

            accel.x() = accelquat.x();
            accel.y() = accelquat.y();
            accel.z() = accelquat.z();
            

            Vector3d accelnorm(accel.normalized());

            float phi = acos(accelnorm.y());

            Vector3d naxis(accelnorm.cross(Vector3d(0,1,0)));

            naxis = naxis.normalized();


            Quaterniond accelrotquat(AngleAxisd((1-alpha)*phi,naxis));

            orientationquat = accelrotquat * orientationquat;


            Vector3d adjaxis(1,0,0);
            Quaterniond quatadj(AngleAxisd(M_PI_2,adjaxis));
            

            Quaterniond orientationquatadj = quatadj*orientationquat;

            Matrix3d R = orientationquat.toRotationMatrix();

            orientationeuler = R.eulerAngles(0,1,2);

            

            currentpacket.r.orientationquatadj = eigentoquatstruct(orientationquatadj);
            currentpacket.r.orientationquat = eigentoquatstruct(orientationquat);
            currentpacket.r.orientationeuler = vector3tofloat(orientationeuler);
            
            return currentpacket;
        }

        Vector3float getworldaccel(navpacket _state){
            Vector3d accelvec = vectorfloatto3(_state.r.imudata.accel);
            Quaterniond orientationquat3 = quatstructtoeigen(_state.r.orientationquatadj);//.inverse();

            Quaterniond accelquat2;

            accelquat2.x() = accelvec.x();
            accelquat2.y() = accelvec.y();
            accelquat2.z() = accelvec.z();

            accelquat2 = orientationquat3 * (accelquat2 * orientationquat3.inverse());

            accelvec.x() = accelquat2.x();
            accelvec.y() = accelquat2.y();
            accelvec.z() = accelquat2.z();   

            Vector3d grav;
            grav << 0,0,9.801;
            Vector3d _accelworld = accelvec-grav;

            return vector3tofloat(_accelworld);

        };

        

};

#endif // NAVCOREHEADER