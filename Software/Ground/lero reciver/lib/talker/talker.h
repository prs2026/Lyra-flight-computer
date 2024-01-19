#if !defined(TALKER)
#define TALKER

#include "AudioFileSourceSD.h"
#include "AudioOutputI2S.h"
#include "AudioGeneratorMP3.h"
#include <SD.h>
#include <macros.h>

File dir;
AudioFileSourceSD *source = NULL;
AudioOutputI2S *output = NULL;
AudioGeneratorMP3 *decoder = NULL;

bool first = true;

class TALKIE
{
private:
    /* data */
public:
    TALKIE(/* args */);
    int saytest();
    int init();
    int run();
    int ispacketinteresting(telepacket lastpacket, telepacket newpacket);
    int sayword(int word);
    int saydigit(int digit);
    int saynum(int num);
};

TALKIE::TALKIE()
{

}

int TALKIE::init(){
    audioLogger = &Serial;  
    source = new AudioFileSourceSD();
    output = new AudioOutputI2S(44100,BCLK,DIN);
    decoder = new AudioGeneratorMP3();
    return 0;
}


int TALKIE::saytest(){
      if ((decoder) && (decoder->isRunning())) {
    if (!decoder->loop()) decoder->stop();
  } else {
    File file = SD.open("/numbers.mp3");
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
    return 0;
}

int TALKIE::saydigit(int digit){
  source->open("/numbers.mp3");
  switch (digit)
  {
  case 0:
    decoder->begin(source,output);
    break;
  
  default:
    break;
  }
  return 0;
}

int TALKIE::sayword(int word){
  return 0;
}

int TALKIE::saynum(int num){
  return 0;
}

int TALKIE::run(){
  if (decoder->isRunning()) {
    if (!decoder->loop()) decoder->stop(); 
  } 
  return 0;
}


int TALKIE::ispacketinteresting(telepacket lastpacket, telepacket newpacket){
  if (lastpacket.r.state = 0 && newpacket.r.state > 0)
  {
    
  }
  return 0;
}





#endif // TALKER