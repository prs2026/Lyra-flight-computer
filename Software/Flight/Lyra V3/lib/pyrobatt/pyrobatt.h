#if !defined(PYROBATTLIB)
#define PYROBATTLIB

#include <generallib.h>


const int enpins[4] = {P1_EN,P2_EN,P3_EN,P4_EN};
const int contpins[4] = {P1_CONT,P2_CONT,P3_CONT,P4_CONT};


class PYROCHANNEL{
int state = 0;
uint32_t firedtime;
uint32_t timeout = 1000;
int identity;

int EN_PIN;
int CONT_PIN;

public:
    int continuity;

    PYROCHANNEL(int _identifier);
    
    void fire();
    void checkfire();

    int getcont();
};


PYROCHANNEL::PYROCHANNEL(int _identifier){
    identity = _identifier-1;
    EN_PIN = enpins[identity];
    CONT_PIN = contpins[identity];
    pinMode(EN_PIN,OUTPUT);
    pinMode(CONT_PIN,INPUT);
}


void PYROCHANNEL::fire(){
    if (state == 0)
    {
        Serial.printf("firing pyro %d\n",identity);
        firedtime = millis();
        // todo: schedule an inturrupt to turn it off?
        digitalWrite(EN_PIN,HIGH);
        state = 1;
    }
    return;
}


void PYROCHANNEL::checkfire(){
    if (state >= 1 && millis()-firedtime > timeout)
    {
        Serial.printf("disabling pyro %d\n",identity);
        digitalWrite(EN_PIN,LOW);
        state = 0;
    }
    return;
}

int PYROCHANNEL::getcont(){
    continuity = digitalRead(CONT_PIN);
    return continuity;
}


#endif //PYROBATTLIB