#if !defined(e22lib)
#define e22lib
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "macros.h"
#include <string.h>
#include <LoRa_E220.h>

#define CONFIG 1
#define NORMAL 0


void printBin(byte aByte) {
  for (int8_t aBit = 7; aBit >= 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
}

class e22simple
{
private:
    int AUXPIN,M1PIN;

public:
    e22simple(){
        1 == 1;
        return;
    }


    void setmode(int mode){
        digitalWrite(M1PIN,mode);
        return;
    }

    // radio must be in config mode, set with setmode()
    void readreg(uint8_t *buf, uint8_t address, uint8_t len){
        setmode(1); 
        uint8_t sendbuf[3] = {0xC1,address,len};

        //Serial.printf("sending: %x %x %x ",sendbuf[0],sendbuf[1],sendbuf[2]);
        Serial1.write(sendbuf,3);
        uint32_t sendtime = millis();
        while (!Serial1.available() && millis() - sendtime < 500)
        {
            delay(10);
            //Serial.print(digitalRead(AUXPIN));
        }
        delay(50);
        int j = 0;
        
        while (Serial1.available() > 1)
        {
            buf[j] = Serial1.read();
            j++;
            delay(10);
        }
        
        return;
    }



    void setreg(uint8_t address, uint8_t value){
        uint8_t sendbuf[4] = {0xC0,address,0x01,value};
        Serial1.write(sendbuf,4);
        uint32_t sendtime = millis();
        while (!Serial1.available() && millis() - sendtime < 500)
        {
            delay(10);
            //Serial.print(digitalRead(AUXPIN));
        }
        delay(50);
        int j = 0;
        uint8_t buf[10] = {};
        while (Serial1.available() > 1)
        {
            buf[j] = Serial1.read();
            j++;
        }
        j = 0;
        
        return;

    }

    void printreg(uint8_t address, uint8_t len){
        
        uint8_t readbuf[15] = {};
        readreg(readbuf,address,len);
        Serial.printf("reg addr 0x%x, len %d\n",address,len);
        Serial.println("hex: bin: dec:");
        int j = 0;
        while (j < 3+len)
        {
            Serial.printf("0x%x, ",readbuf[j]);
            printBin(readbuf[j]);
            Serial.printf(", %d\n",readbuf[j]);
            j++;
        }
        
        return;
    }

    

};



#endif // e22lib