#include <Arduino.h>


#include <SPI.h>                                 //the lora device is SPI based so load the SPI library
#include <SX128XLT.h>                            //include the appropriate library   
#include <basiclib.h>

stationdata Station1;
stationdata Station2;
stationdata Station3;

#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         

#include "basiclib.h"
#include "sx1280lib.h"
#include <gpslib.h>
#include <ArduinoEigenDense.h>

#define MODEFLIGHT
//#define MODESTATION

#if !defined(MODEFLIGHT)
#if !defined(MODESTATION)
#define MODEGROUND

#endif // MODE FLIGHT
#endif // MODE STATION
sx1280radio radio;
gpsinput gps;

const uint32_t checkinterval = 500;
uint32_t checktime;

const uint32_t sendpacketinterval = 2000;
uint32_t sendpackettime;

Matrix3d stationpoints;



void setup( ) {
  delay(3000);
  Serial.begin();
  Serial.println("init");
  pinMode( PIN_LED, OUTPUT);

  pinMode( PIN_TXCOEN, OUTPUT );
  digitalWrite( PIN_TXCOEN, HIGH );

  radio.initradio();

  Serial1.setRX(UART_RX_PIN);
  Serial1.setTX(UART_TX_PIN);

  Serial1.begin(9600);
 
  Station1.ID = 16;
  Station1.alt = 5029;
  Station1.lat = 39.24715773473315;
  Station1.lon = -119.81061508627381;
  Station1.distance = 1940;
  Station1.xcoord = 0;
  Station1.ycoord = 0;

  Station2.ID = 17;
  Station2.alt = 5029;
  Station2.lat = 39.275649549942784;
  Station2.lon = -119.79315491913076;
  Station2.distance = 2370;

  Station3.ID = 18;
  Station3.alt = 5029;
  Station3.lat = 39.238545400773376;
  Station3.lon = -119.76962665183977;
  Station3.distance = 2510;

  // test point 39.25442283421696, -119.79019707259835, 5029 (on surface of lake)

  #if defined(MODEFLIGHT)
  Serial.print("flightuni");
  radio.setuptorange(0x01);
  
  pinMode(UART_TX_PIN,OUTPUT);
  digitalWrite(UART_TX_PIN, HIGH);

  //setup reference plane

  Eigen::Vector3d station2pos = gpsToENU(Station2.lat,Station2.lon,Station2.alt,Station1.lat,Station1.lon,Station1.alt);

  Station2.xcoord = station2pos.x();
  Station2.ycoord = station2pos.y();
  Station2.zcoord = station2pos.z();

  Serial.printf("Station 2 coords: %f %f %f\n",Station2.xcoord,Station2.ycoord,Station2.zcoord);

  Eigen::Vector3d station3pos = gpsToENU(Station3.lat,Station3.lon,Station3.alt,Station1.lat,Station1.lon,Station1.alt);

  Station3.xcoord = station3pos.x();
  Station3.ycoord = station3pos.y();
  Station3.zcoord = station3pos.z();

  Serial.printf("Station 3 coords: %f %f %f\n",Station3.xcoord,Station3.ycoord,Station3.zcoord);

  stationpoints << 0,0,0,
                  station2pos.x(),station2pos.y(),station2pos.z(),
                  station3pos.x(),station3pos.y(),station3pos.z();

  Serial.println("trying to trilaterate");
  uint32_t mathcalctime = micros();

  Vector3d testmatrix(Station1.distance,Station2.distance,Station3.distance);

  Vector3d solution1, solution2;
  
  

  if (trilaterate(stationpoints, testmatrix, solution1, solution2)) {
        //Serial.printf("Solution 1: %f %f %f\n",solution1.x(),solution1.y(),solution1.z());
        //Serial.printf("Solution 2: %f %f %f\n",solution2.x(),solution2.y(),solution2.z());
    } else {
        //Serial.println("no valid solution");
    }

  double outlat, outlon, outalt;
  
  enuToLLA(solution2,Station1.lat,Station1.lon,Station1.alt,outlat,outlon,outalt);
  Serial.printf("Took %d microseconds",micros()-mathcalctime);
  Serial.printf("Solution 2 gps point: lat/lon: %f,%f alt %f\n",outlat,outlon,outalt);

  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)

  Serial.print("pingstation");

  radio.setuptorange(0x00);
  radio.settolisten(ThisStationAddress);
  
  #endif // MODEGROUND
  #if defined(MODESTATION)

  Serial.print("groundstation");
  
  #endif // MODEGROUND

}

void loop() {
  #if defined(MODEFLIGHT)

  packet packettosendloop;

  Station1.distance = radio.pingrange(Station1.ID);
  Station2.distance = radio.pingrange(Station2.ID);
  Station3.distance = radio.pingrange(Station3.ID);

  Vector3d distancematrix(Station1.distance,Station2.distance,Station3.distance);


  Vector3d solution1, solution2;
  
  Serial.println("trying to trilaterate");

  if (trilaterate(stationpoints, distancematrix, solution1, solution2)) {
        Serial.printf("Solution 1: %f %f %f\n",solution1.x(),solution1.y(),solution1.z());
        Serial.printf("Solution 2: %f %f %f\n",solution2.x(),solution2.y(),solution2.z());
    } else {
        Serial.println("no valid solution");
    }

  double outlat, outlon, outalt;
  
  enuToLLA(solution2,Station1.lat,Station1.lon,Station1.alt,outlat,outlon,outalt);
  
  Serial.printf("Solution 2 gps point: lat/lon: %f,%f alt %f\n",outlat,outlon,outalt);

  packettosendloop.r.lat = outlat;
  packettosendloop.r.lat = outlon;
  packettosendloop.r.alt = outalt;
  packettosendloop.r.battvoltage = getbatteryvoltage();

  packettosendloop.r.uptime = millis();
  
  packettosendloop.r.command = 0xa;
  
  radio.sendpacket(packettosendloop);

  //delay(1000);
  
  #endif // MODEFLIGHT
  
  #if defined(MODEGROUND)

  if(millis() - checktime > checkinterval){

    checktime = millis();
    radio.checkforping();
  }

  

  //gps.checkformessages();
  
  #endif // MODEGROUND

  #if defined(MODESTATION)
  
  packet newpacket = radio.receivepacket();

  Serial.printf("\n>uptime: %d\n", newpacket.r.uptime);
  Serial.printf(">lat: %f\n", newpacket.r.lat);
  Serial.printf(">lon: %f\n", newpacket.r.lon);
  Serial.printf(">battvoltage: %f\n", newpacket.r.battvoltage);
  #endif // MODESTATION
  

  
}

