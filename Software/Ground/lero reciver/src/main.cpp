#include <Arduino.h>
#include <MACROS.h>
#include <SPI.h>
#include <SD.h>

#include <radio.h>
#include <talker.h>

#define SPI_SPEED SD_SCK_MHZ(4)

telepacket currentstate;

TALKIE talkie;
RADIO radio;


void recivepacket()

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
  Serial.println("serial init");

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

  radio.init();



  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.println("out of setup");
  
  
  
}

void loop() {
  //talkie.saytest();
  if (Serial1.available())
  {
    Serial.printf("new radio message %d",Serial1.read());
  }
  
}