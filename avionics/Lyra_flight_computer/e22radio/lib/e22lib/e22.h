#if !defined(e22lib)
#define e22lib
#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "macros.h"
#include <string.h>

#define CONFIG 1
#define NORMAL 0


class e22
{
private:
    int M1PIN;
public:
    e22(int _m1pin){
        M1PIN = _m1pin;
    }

    void setmode(int mode){
        digitalWrite(M1PIN,mode);
        return;
    }

    int setup(){

        Serial1.begin(9600);
        
        pinMode(M1PIN,OUTPUT);
        digitalWrite(M1PIN,LOW);
        uint8_t recivebuf[8];
        setmode(CONFIG);
        Serial.println("recived:");
        readreg(recivebuf,0x00,2);
        return 0;
    }
    
    // radio must be in config mode, set with setmode()
    void readreg(uint8_t *buf, uint8_t address, uint8_t len){ 
        uint8_t sendbuf[3] = {0xC1,address,len};

        Serial.printf("sending: %x %x %x",sendbuf[0],sendbuf[1],sendbuf[2]);
        Serial1.write(sendbuf,3);
        uint32_t sendtime = millis();
        while (Serial1.available() == 0 && millis() - sendtime < 500)
        {
            delay(1);
        }
        uint8_t readbuf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        int j = 0;
        Serial.print("\nrecived: ");
        while (Serial1.available() > 1)
        {
            readbuf[j] = Serial1.read();
            Serial.print(readbuf[j],HEX);
            Serial.print(" ");
            j++;
        }

        
        return;
    }

};



#endif // e22lib