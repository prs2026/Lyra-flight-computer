#include <Arduino.h>
#include <MACROS.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <SD.h>
#include <E220.h>

Stream &radioserial = (Stream &)Serial1;
                    //  m0         m1     aux
E220 ebyte(&radioserial,11,26,11);


void setup(void) {

  pinMode(LCDDI,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(115200);
  while (!Serial);
  delay(500);
  Serial.println("init");

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

  File dataFile = SD.open("/datalog.txt", FILE_WRITE);
  if (!dataFile)
  {
    Serial.println("SD file init fail");
  }
  dataFile.print("testing");
  dataFile.close();
  
  Serial1.end();
  Serial1.setRX(11);
  Serial1.setTX(11);
  Serial1.begin(9600);



  uint32_t inittime = millis();    
  while (millis()-inittime < 1000)
  {
      if (ebyte.init())
      {
          Serial.println("radio init sucess");
          break;
      }
      Serial.println("radio init attempt fail");
      
  }

  ebyte.setAddress(0x1234,true);
  ebyte.setPower(Power_21,true);
  ebyte.setChannel(68,true);
  ebyte.setNetID(0,true);
  ebyte.setBaud(UDR_9600,true);
  ebyte.setSubPacketSize(SPS_64,true);
  ebyte.setAirDataRate(ADR_2400,true);
  ebyte.setEncryptionKey(0,true);
  ebyte.setLBT(true,true);
  ebyte.setFixedTransmission(true,true);
  ebyte.printBoardParameters();
  

  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.println("out of setup");
}

void loop() {
  // u8g2.firstPage();
  // do {
  //   u8g2.setFont(u8g2_font_ncenB14_tr);
  //   u8g2.drawStr(0,24,"Hello World!");
  // } while ( u8g2.nextPage() );
}