#include <Arduino.h>
#include <basiclib.h>


int parsecommand(uint8_t command){
    switch (command)
    {
    case 0x4:

        Serial.println("triggering camera");
        digitalWrite(UART_TX_PIN,LOW);
        delay(1000);
        digitalWrite(UART_TX_PIN, HIGH);

        // packet testpacket;
        // testpacket.r.uptime = millis();
        // testpacket.r.command = 0xFE;
        // testpacket.r.battvoltage = 80.0;
        // testpacket.r.checksum = 0xAB;
        // testpacket.r.lat = 24.5;
        // testpacket.r.lat = 117.5;
        // radio.sendpacket(testpacket);
        break;
    
    default:
        break;
    }
    return 0;
}

float getbatteryvoltage(){
    int rawvalue = analogRead(PIN_BATTSENSE);

    int mappedvalue = map(rawvalue,0,1023,0,330);

    float realval = float(mappedvalue)/(51.0/(51.0+30.0));

    return realval/100;

}