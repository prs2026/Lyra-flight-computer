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



class e22
{
private:
    int AUXPIN,M1PIN;

public:
    e22(int auxpin, int _m1pin){
        AUXPIN = auxpin;
        M1PIN = _m1pin;
    }


    void setmode(int mode){
        digitalWrite(M1PIN,mode);
        return;
    }

    // radio must be in config mode, set with setmode()
    void readreg(uint8_t *buf, uint8_t address, uint8_t len){ 
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
        for (int i = 0; i < 4+len; i++)
        {
            Serial.printf("0x%x, ",readbuf[j]);
            printBin(readbuf[j]);
            Serial.printf(", %d\n",readbuf[j]);
            j++;
        }

    }

    int setup(){

        
        
        pinMode(M1PIN,OUTPUT);
        digitalWrite(M1PIN,LOW);

        pinMode(AUXPIN,INPUT);
        //Serial.print(digitalRead(AUXPIN));

        uint8_t recivebuf[10];
        setmode(CONFIG);
        Serial1.begin(9600);

        //Serial.println("recived:");
        //Serial.print(digitalRead(AUXPIN));
        printreg(0x00,2);
        printreg(0x03,2);
        printreg(0x05,2);
        setreg(0x05,68);
        printreg(0x05,2);
        setreg(0x00,0xFF);
        setreg(0x00,0xFF);
        setreg(0x04,0b11000000);
        printreg(0x00,3);
        setmode(0);
        Serial1.write(0x32);
        return 0;
    }
    
    

};



#endif // e22lib