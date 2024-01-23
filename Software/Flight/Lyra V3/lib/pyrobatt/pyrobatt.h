#if !defined(PYROBATTLIB)
#define PYROBATTLIB

#include <generallib.h>


const int enpins[4] = {P1_EN,P2_EN,P3_EN,P4_EN};
const int contpins[4] = {P1_CONT,P2_CONT,P3_CONT,P4_CONT};


class PYROCHANNEL{
int state;
uint32_t firedtime;
uint32_t timeout = 500;
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
}


void PYROCHANNEL::fire(){
    if (state = 0)
    {
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