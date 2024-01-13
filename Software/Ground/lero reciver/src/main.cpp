#include <Arduino.h>
#include <MACROS.h>
#include <SPI.h>
#include <SD.h>

#include "AudioFileSourceSD.h"
#include "AudioOutputI2S.h"
#include "AudioGeneratorMP3.h"

#include <radio.h>

#define SPI_SPEED SD_SCK_MHZ(4)


File dir;
AudioFileSourceSD *source = NULL;
AudioOutputI2S *output = NULL;
AudioGeneratorMP3 *decoder = NULL;
bool first = true;

RADIO radio;

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

  audioLogger = &Serial;  
  source = new AudioFileSourceSD();
  output = new AudioOutputI2S(44100,BCLK,DIN);
  decoder = new AudioGeneratorMP3();


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

  radio.init();



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