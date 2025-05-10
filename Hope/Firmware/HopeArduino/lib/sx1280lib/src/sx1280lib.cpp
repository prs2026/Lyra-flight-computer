#include "Arduino.h"
#include "SPI.h"
#include "sx1280lib.h"

sx1280radio::sx1280radio(uint8_t cssPin, 
                    uint8_t busyPin, 
                    uint8_t resetPin){
        _CS_PIN = cssPin;
        _BUSY_PIN = busyPin;
        _RESET_PIN = resetPin;
    }

int sx1280radio::initradio(){
    pinMode( _CS_PIN, OUTPUT );
    pinMode( _BUSY_PIN, OUTPUT );
    pinMode( _RESET_PIN, INPUT );

    digitalWrite(_CS_PIN, HIGH);
    digitalWrite(_RESET_PIN, HIGH);
    reset();

    while (isbusy())
    {
        delay(10);
        Serial.println("busy at init");
    }
    return 1;
}

sx1280radio::~sx1280radio(){
    return;
}

void sx1280radio::reset(){
    digitalWrite(_RESET_PIN, 0);
    delay(10);
    digitalWrite(_RESET_PIN, 1);
    return;
}

int sx1280radio::isbusy(){
    return digitalRead(_BUSY_PIN);
}

int sx1280radio::sendcommand(uint8_t opcode,uint8_t data[],uint8_t len)
{
    while (isbusy())
    {
        delay(10);
        Serial.println("waiting to send command ");
    }
    
    uint8_t received[len];
    digitalWrite(_CS_PIN,0);
    received = SPI1.transfer(data,len);
    digitalWrite(_CS_PIN,1);
}