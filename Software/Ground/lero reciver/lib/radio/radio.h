#if !defined(LYRAV2SENSORSLIB)
#define LYRAV2SENSORSLIB
#include <macros.h>

Stream &radioserial = (Stream &)Serial1;
                    //  m0         m1     aux
E220 ebyte(&radioserial,RADIOM0,RADIOM1,RADIOAUX);


class RADIO{

    uint16_t groundaddress = 0x1234; 
    uint16_t airaddress = 0xABCD; 

    uint8_t radiochannel = 68;

    public:

    int init();
};

int RADIO::init(){
        Serial.println("radio start");

        Serial1.end();
        Serial1.setRX(RADIORX);
        Serial1.setTX(RADIOTX);
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
        return 0;
}


#endif // LYRAV2SENSORSLIB