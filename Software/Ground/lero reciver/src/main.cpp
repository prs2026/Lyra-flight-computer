#include <Arduino.h>
#include <MACROS.h>
#include <SPI.h>
#include <SD.h>
<<<<<<< Updated upstream
#include <E220.h>

Stream &radioserial = (Stream &)Serial1;
                    //  m0         m1     aux
E220 ebyte(&radioserial,11,26,11);
=======

#include "AudioFileSourceSD.h"
#include "AudioOutputI2S.h"
#include "AudioGeneratorMP3.h"

#define SPI_SPEED SD_SCK_MHZ(4)


File dir;
AudioFileSourceSD *source = NULL;
AudioOutputI2S *output = NULL;
AudioGeneratorMP3 *decoder = NULL;
bool first = true;
>>>>>>> Stashed changes


void setup(void) {

  pinMode(LRC,OUTPUT);
  pinMode(CS_SD,OUTPUT);
  digitalWrite(CS_SD,HIGH);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LRC,HIGH);

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(115200);
  while (!Serial);
  delay(500);
  Serial.println("init");

<<<<<<< Updated upstream
=======
  audioLogger = &Serial;  
  source = new AudioFileSourceSD();
  output = new AudioOutputI2S(44100,BCLK,DIN);
  decoder = new AudioGeneratorMP3();


>>>>>>> Stashed changes
  SPI.setTX(MOSISD);
  SPI.setRX(MISOSD);
  SPI.setSCK(SCKSD);
  SPI.begin();
  SPI.end();

<<<<<<< Updated upstream
  // Serial.println("SPI init");

  // if (!SD.begin(CS_SD,SPI))
  // {
  //   Serial.println("SD init fail");
  // }

  // File dataFile = SD.open("/datalog.txt", FILE_WRITE);
  // if (!dataFile)
  // {
  //   Serial.println("SD file init fail");
  // }
  // dataFile.print("testing");
  // dataFile.close();
  
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
  
=======
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


>>>>>>> Stashed changes

  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.println("out of setup");


  
}

void loop() {
  if ((decoder) && (decoder->isRunning())) {
    if (!decoder->loop()) decoder->stop();
  } else {
    File file = SD.open("/hi.mp3");
    if (file) { 
      if (!first) {
        source->close();
        if (source->open(file.name())) { 
          Serial.printf_P(PSTR("Playing '%s' from SD card...\n"), file.name());
          decoder->begin(source, output);
        } else {
          Serial.printf_P(PSTR("Error opening '%s'\n"), file.name());
        }
      }else first = false;
    } else {
      Serial.println(F("Playback from SD card done\n"));
      delay(1000);
    }       
  }
}