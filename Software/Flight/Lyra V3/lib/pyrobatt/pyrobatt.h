#if !defined(PYROBATTLIB)
#define PYROBATTLIB

#include <generallib.h>

class PYROCHANNEL{
int state;
uint32_t firedtime;
uint32_t timeout;
int identity;

int enpins[4] = {P1_EN,P2_EN,P3_EN,P4_EN};
int contpins[4] = {P1_CONT,P2_CONT,P3_CONT,P4_CONT};

int EN_PIN;
int CONT_PIN;

public:
    int continuity;

    PYROCHANNEL(int _identifier);
    
    void fire(uint32_t preset);


    int getcont();
};


PYROCHANNEL::PYROCHANNEL(int _identifier){
    identity = _identifier;
    EN_PIN = enpins[identity];
    CONT_PIN = contpins[identity];
}


void PYROCHANNEL::fire(uint32_t preset){
    firedtime = millis();
    // todo: schedule an inturrupt to turn it off?
    digitalWrite(EN_PIN,HIGH);
    state = 1;
    return;
}



int PYROCHANNEL::getcont(){
    continuity = digitalRead(CONT_PIN);
    return continuity;
}


#endif //PYROBATTLIB