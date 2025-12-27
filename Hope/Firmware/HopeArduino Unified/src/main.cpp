#include <Arduino.h>

#include <SPI.h>                                 //the lora device is SPI based so load the SPI library
#include <SX128XLT.h>                            //include the appropriate library   
#include <basiclib.h>

stationdata Station1;
stationdata Station2;
stationdata Station3;

stationdata ThisStation;

#include <SPI.h>                                //the lora device is SPI based so load the SPI library                                         

#include "basiclib.h"
#include "sx1280lib.h"
#include <gpslib.h>
#include <ArduinoEigenDense.h>

//#define MODEFLIGHT

//#define DATARECEIVER

#if !defined(MODEFLIGHT)
#if !defined(DATARECEIVER)
#define PINGSTATION

#endif // MODE FLIGHT
#endif // MODE STATION
sx1280radio radio;
gpsinput gps;

const uint32_t checkinterval = 500;
uint32_t checktime;

const uint32_t sendpacketinterval = 2000;
uint32_t sendpackettime;

const uint32_t sendpositioninterval = 30000;
uint32_t sendpositiontime;

Matrix3d stationpoints;

Vector3d lastpoint = Vector3d::Zero();
uint32_t lastpointtime = 0;

void setup( ) {
  delay(3000);
  Serial.begin(115200);
  Serial.println("init");
  pinMode( PIN_LED, OUTPUT);

  pinMode( PIN_TXCOEN, OUTPUT );
  digitalWrite( PIN_TXCOEN, HIGH );

  radio.initradio();

  Serial1.setRX(UART_RX_PIN);
  Serial1.setTX(UART_TX_PIN);
  Serial1.begin(9600);

  //all at 6400ft/1950.6m

  //station 1 39.251663002648804, -119.96871477918272
 
  Station1.ID = 16;
  Station1.alt = 1950.6; // m
  Station1.lat = 39.251663002648804;
  Station1.lon = -119.96871477918272;
  Station1.distance = 162;
  Station1.xcoord = 0;
  Station1.ycoord = 0;

  //station2 39.251512936807316, -119.96958783820142

  Station2.ID = 17;
  Station2.alt = 1950.6; // ft
  Station2.lat = 39.251512936807316;
  Station2.lon = -119.96958783820142;
  Station2.distance = 177;

  //station3 39.25232642482777, -119.96894920782334

  Station3.ID = 18;
  Station3.alt = 1950.6; // ft
  Station3.lat = 39.25232642482777;
  Station3.lon = -119.96894920782334;
  Station3.distance = 176;

  ThisStation = Station2; // CHANGE THIS WHEN UPLOADING TO PING STATIONS

  
  #if defined(MODEFLIGHT)
  Serial.print("flightuni");
  radio.setuptorange(0x01);
  
  pinMode(UART_TX_PIN,OUTPUT);
  digitalWrite(UART_TX_PIN, HIGH);

  

  #endif // MODEFLIGHT
  
  #if defined(PINGSTATION)

  Serial.print("pingstation");
  Serial.printf("THIS STATION ID: %d",ThisStation.ID);

  radio.setuptorange(0x00);
  radio.settolisten(ThisStation.ID); 
  
  #endif // PINGSTATION

  
  #if defined(DATARECEIVER)

  Serial.print("groundstation");

  

  // test point 39.25189334300624, -119.96920743754161, 6400 (on surface of lake)


  // setup reference plane

  Eigen::Vector3d station2pos = gpsToENU(Station2.lat,Station2.lon,Station2.alt,Station1.lat,Station1.lon,Station1.alt);

  Station2.xcoord = station2pos.x();
  Station2.ycoord = station2pos.y();
  Station2.zcoord = station2pos.z();

  //Serial.printf("Station 2 coords: %f %f %f\n",Station2.xcoord,Station2.ycoord,Station2.zcoord);

  Eigen::Vector3d station3pos = gpsToENU(Station3.lat,Station3.lon,Station3.alt,Station1.lat,Station1.lon,Station1.alt);

  Station3.xcoord = station3pos.x();
  Station3.ycoord = station3pos.y();
  Station3.zcoord = station3pos.z();

  //Serial.printf("Station 3 coords: %f %f %f\n",Station3.xcoord,Station3.ycoord,Station3.zcoord);

  stationpoints << 0,0,0,
                  station2pos.x(),station2pos.y(),station2pos.z(),
                  station3pos.x(),station3pos.y(),station3pos.z();

  Serial.println("trying to trilaterate");
  //uint32_t mathcalctime = micros();

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
  //Serial.printf("Took %d microseconds",micros()-mathcalctime);
  Serial.printf("Solution 2 gps point: lat/lon: %f,%f alt %f\n",outlat,outlon,outalt);
  
  #endif // PINGSTATION

}

void loop() {
  //------------------------------------------------------------------------------
  #if defined(MODEFLIGHT)

  radio.setuptorange(0x01);

  Station1.distance = radio.pingrange(Station1.ID);
  Station2.distance = radio.pingrange(Station2.ID);
  Station3.distance = radio.pingrange(Station3.ID);

  

  packet packettosendloop;
  packettosendloop.r.checksum = 101; // flight unit identifier
  packettosendloop.r.lat = Station1.distance;
  packettosendloop.r.lon = Station2.distance;
  packettosendloop.r.alt = Station3.distance;
  packettosendloop.r.battvoltage = getbatteryvoltage()*1e2;

  packettosendloop.r.uptime = millis();

  radio.sendpacket(packettosendloop);

  //delay(1000);
  
  #endif // MODEFLIGHT
  //------------------------------------------------------------------------------
  #if defined(PINGSTATION)

  if(millis() - checktime > checkinterval){

    checktime = millis();
    radio.checkforping();
  }

  gps.checkformessages();
  //Serial.println("checking gps");
  
  if (/*gps.hasfix()*/ 1 & (millis() - sendpositiontime > sendpositioninterval))
  {
    Serial.print("sending gps location\n");
    sendpositiontime = millis();
    Vector3d currentpos;
    currentpos = gps.getposition();
    packet packettosend;

    packettosend.r.lat = currentpos.x()*1e6;
    packettosend.r.lon = currentpos.y()*1e6;
    packettosend.r.alt = currentpos.z()*1e6;
    packettosend.r.battvoltage = getbatteryvoltage()*1e2;
    packettosend.r.uptime = millis();
    packettosend.r.checksum = ThisStation.ID;

    radio.sendpacket(packettosend);

    radio.setuptorange(0x00);
    radio.settolisten(ThisStation.ID);
  }
  
  #endif // PINGSTATION
  //------------------------------------------------------------------------------
  #if defined(DATARECEIVER)
  
  recievedpacket newpacket = radio.receivepacket();

  if (newpacket.checksum == 101)
  {
    Vector3d distancematrix(newpacket.distance1,newpacket.distance2,newpacket.distance3);

    Vector3d solution1, solution2;
    
    Serial.println("trying to trilaterate");

    if (trilaterate(stationpoints, distancematrix, solution1, solution2)) {
          Serial.printf("Solution 1: %f %f %f\n",solution1.x(),solution1.y(),solution1.z());
          Serial.printf("Solution 2: %f %f %f\n",solution2.x(),solution2.y(),solution2.z());
    } else {
        Serial.println("no valid solution");
    }

    double outlat = 0;
    double outlon = 0;
    double outalt = 0;

    float solvedvvel = 0;
    float solveddistance = 1200;

    solvedvvel = (solution1.z() - lastpoint.z())/(float(millis() - lastpointtime)/1e3);

    lastpoint = solution1;
    lastpointtime = millis();
    
    enuToLLA(solution2,Station1.lat,Station1.lon,Station1.alt,outlat,outlon,outalt);
    
    //Serial.printf("Solution 2 gps point: lat/lon: %f,%f alt %f\n",outlat,outlon,outalt);

    

    Serial.printf("\n %f,%f,%f,%f,%d,%d,%d,%f,%d,%d,%f,%f \n",float(newpacket.uptime)/1e3,
                                                        outalt,
                                                        solvedvvel,
                                                        solveddistance,
                                                        newpacket.distance1,
                                                        newpacket.distance2,
                                                        newpacket.distance3,
                                                        float(newpacket.battvoltage)/1e2,
                                                        newpacket.rssi,
                                                        newpacket.snr,
                                                        outlat,
                                                        outlon);
  }
  else if (newpacket.checksum == Station1.ID)
  {
    Station1.lat = newpacket.distance1/1e6;
    Station1.lon = newpacket.distance2/1e6;
    Station1.alt = newpacket.distance3/1e6;
    Station1.battvoltage = float(newpacket.battvoltage)/1e2;
    Station1.uptime = newpacket.uptime;
    Serial.printf("Station 1 updated data: %f,%f,%f,%f,%f\n",Station1.lat,Station1.lon,Station1.alt,Station1.battvoltage,Station1.uptime);
  }
  else if (newpacket.checksum == Station2.ID)
  {
    Station2.lat = newpacket.distance1/1e6;
    Station2.lon = newpacket.distance2/1e6;
    Station2.alt = newpacket.distance3/1e6;
    Station2.battvoltage = float(newpacket.battvoltage)/1e2;
    Station2.uptime = newpacket.uptime;
    Serial.printf("Station 2 updated data: %f,%f,%f,%f,%f\n",Station2.lat,Station2.lon,Station2.alt,Station2.battvoltage,Station2.uptime);
  }
  else if (newpacket.checksum == Station3.ID)
  {
    Station3.lat = newpacket.distance1/1e6;
    Station3.lon = newpacket.distance2/1e6;
    Station3.alt = newpacket.distance3/1e6;
    Station3.battvoltage = float(newpacket.battvoltage)/1e2;
    Station3.uptime = newpacket.uptime;
    Serial.printf("Station 3 updated data: %f,%f,%f,%f,%f\n",Station3.lat,Station3.lon,Station3.alt,Station3.battvoltage,Station3.uptime);
  }
  
  //------------------------------------------------------------------------------
  #endif // DATARECEIVER
  
}

