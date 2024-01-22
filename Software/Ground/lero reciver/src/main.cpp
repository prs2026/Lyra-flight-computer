#include <Arduino.h>
#include <MACROS.h>
#include <SPI.h>
#include <SD.h>
#include <LittleFS.h>
#include "pico/stdlib.h"

#include <radio.h>
#include <talker.h>

#include <Wire.h>


#include "SerialTransfer.h"


SerialTransfer myTransfer;

#define SPI_SPEED SD_SCK_MHZ(4)

telepacket currentpacket;
telepacket prevpacket;

datapacket testpacket;


TALKIE talkie;
RADIO radio;

uint32_t packettime = 0;
uint32_t senddatatime;

void recivepacket(){
  if (!Serial1.available())
  {
    Serial1.println("no packet");
    return;
  }
  if (Serial1.peek() != 0x12)
  {
    Serial.printf("invalid packet, expected 0x12 got 0x%x \n",Serial1.read());
    return;
  }
  uint8_t databuf[32] = {};

  Serial1.readBytes(databuf,sizeof(databuf));

  if (databuf[0] != 0x12 && databuf[31] != 0x34)
  {
    Serial.println("invalid packet, markers didnt pass");
    return;
  }

  Serial.println("good data packet");
  int j = 0;
  for (int i = 0; i < 32; i++)
  {
    currentpacket.data[j] = databuf[j];
    j++;
  }
  packettime = millis();
  File logfile = SD.open("/log.csv",FILE_WRITE);


  int error = logfile;
  if (!error)
  {
    Serial.println("CANNOT OPEN SD CARD, CHECK CONNECTION");
  }
  

  logfile.printf("AB,"
  "%f,%f,%f," // orientation euler
  "%f,%f,%f," // accel
  "%f,%f,%f," // gyro
  "%f," //alt
  "%f," // vvel
  "%d," // uptime lyra
  "%d," // errorflag MP
  "%d," // errorflag NAV
  "%d," // state
  "%d," // uptime reciver
  "%d," // datage
  "CD\n"
  ,(float(currentpacket.r.orientationeuler.x)/100)*(180/PI),(float(currentpacket.r.orientationeuler.y)/100)*(180/PI),(float(currentpacket.r.orientationeuler.z)/100)*(180/PI)
  ,float(currentpacket.r.accel.x)/100,float(currentpacket.r.accel.y)/100,float(currentpacket.r.accel.z)/100
  ,float(currentpacket.r.gyro.x)/100,float(currentpacket.r.gyro.y)/100,float(currentpacket.r.gyro.z)/100
  ,float(currentpacket.r.altitude)/100
  ,float(currentpacket.r.verticalvel)/100
  ,currentpacket.r.uptime
  ,currentpacket.r.errorflagmp
  ,currentpacket.r.errorflagnav
  ,currentpacket.r.state
  ,millis()
  ,millis() - packettime
  );
  logfile.close();
  packettime = millis();
    // uint8_t checksum;
    // Vector3int orientationeuler;
    // Vector3int accel;
    // Vector3int gyro;
    // int16_t altitude;
    // int16_t verticalvel;
    // uint32_t uptime;
    // uint8_t errorflagmp;
    // uint8_t errorflagnav;
    // uint8_t state;
    // uint8_t checksum2;
}

void senddataover(telepacket sendpacket){
  //myTransfer.sendDatum(sendpacket);
  

  Serial2.printf("A,%d,%d,%d,%d,D,",sendpacket.r.altitude,sendpacket.r.verticalvel,sendpacket.r.state,millis()-packettime);
  // Serial2.printf("A,"
  // "%d,%d,%d," // orientation euler
  // "%d,%d,%d," // accel
  // "%d,%d,%d," // gyro
  // "%d," //alt
  // "%d," // vvel
  // "%d," // uptime lyra
  // "%d," // errorflag MP
  // "%d," // errorflag NAV
  // "%d," // state
  // "%d," // uptime reciver
  // "%d," // datage
  // "D,\n"
  // ,sendpacket.r.orientationeuler.x,sendpacket.r.orientationeuler.y,sendpacket.r.orientationeuler.z
  // ,sendpacket.r.accel.x,sendpacket.r.accel.y,sendpacket.r.accel.z
  // ,sendpacket.r.gyro.x,sendpacket.r.gyro.y,sendpacket.r.gyro.z
  // ,sendpacket.r.altitude
  // ,sendpacket.r.verticalvel
  // ,sendpacket.r.uptime
  // ,sendpacket.r.errorflagmp
  // ,sendpacket.r.errorflagnav
  // ,sendpacket.r.state
  // ,millis()
  // ,millis() - packettime
  // );
  
  //Serial.print("senddata");
}

void setup(void) {

  pinMode(LRC,OUTPUT);
  pinMode(CS_SD,OUTPUT);
  digitalWrite(CS_SD,HIGH);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LRC,HIGH);

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(115200);
  uint32_t starttime = millis();
  while (!Serial && millis()-starttime < 5000);
  delay(500);
  Serial.println("init");

  Serial2.setTX(NANOTX);
  Serial2.setRX(9);
  Serial.println("serial pins set");
  Serial2.begin(9600);
  Serial.println("9600 init");

  SPI.setTX(MOSISD);
  SPI.setRX(MISOSD);
  SPI.setSCK(SCKSD);
  SPI.begin();
  SPI.end();

  Serial.println("SPI init");

  if (!SD.begin(CS_SD,SPI))
  {
    Serial.println("SD init fail");
  }

  talkie.init();

  File dataFile = SD.open("/datalog.txt", FILE_WRITE);
  if (!dataFile)
  {
    Serial.println("SD file init fail");
  }
  dataFile.print("testing");
  dataFile.close();

  File logfile = SD.open("/log.csv", FILE_WRITE);
  if (!logfile)
  {
    Serial.println("SD file init fail");
  }
  logfile.print("checksum, accelx, accely, accel z, gyro x, gyro y, gyro z, altitude, vvel, lyra uptime, errorflag MP, errorflag NAV, state, uptime reciver, dataage,checksum2");
  logfile.close();

  radio.init();

  myTransfer.begin(Serial2);

  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.println("out of setup");
  
  
  
}

void loop() {
  //talkie.saytest();
  if (Serial1.available())
  {
    Serial.println("new radio message");
    recivepacket();
    talkie.ispacketinteresting(prevpacket,currentpacket);
    prevpacket = currentpacket;
    //senddataover(currentpacket);
  }
  if (millis()-senddatatime > 250)
  {
    senddataover(currentpacket);
    senddatatime = millis();
  }
  
  talkie.run();
}