#include <BASIC.h>
#include <Arduino.h>

int setuppins(){
    pinMode(BUZZERPIN,OUTPUT);
    pinMode(LEDPIN, OUTPUT);

    pinMode(P4EN,OUTPUT);
    pinMode(P4CONT,INPUT);

    pinMode(P3EN,OUTPUT);
    pinMode(P3CONT,INPUT);

    pinMode(P2EN,OUTPUT);
    pinMode(P2CONT,INPUT);

    pinMode(P1EN,OUTPUT);
    pinMode(P1CONT,INPUT);

    pinMode(BRKOUT1,INPUT);
    pinMode(BRKOUT2,INPUT);
    pinMode(BRKOUT3,INPUT);
    pinMode(BRKOUT4,INPUT);
    return 0;
}

int beep(uint32_t duration,uint16_t freq){
    tone(BUZZERPIN,freq,duration);
    return 0;
}

int setled(int state){
    digitalWrite(LEDPIN,!state);
    return 0;
}

//scans the i2c bus, returns 1 if none are found, otherwise 0
int scani23c(bool printout){
    byte error, address;
    int nDevices;
     printout ? Serial.print("Scanning..")  : 0;
    
    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0){
        printout ? Serial.print("I2C device found at address 0x") : 0;
        if (address<16)
            Serial.print("0");
        printout ? Serial.print(address,HEX) : 0;
        printout ? Serial.println("  !") : 0;
    
        nDevices++;
        }
        else if (error==4){
        printout ? Serial.print("Unknown error at address 0x") : 0;
        if (address<16)
            printout ? Serial.print("0") : 0;
            printout ? Serial.println(address,HEX) : 0;
        }    
    }
    if (nDevices == 0){
        printout ? Serial.println("No I2C devices found\n") : 0;
        return 1;
    }
    else
        printout ? Serial.println("done\n") : 0;
    
    return 0;
}

