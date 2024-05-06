#if !defined(PYROBATTLIB)
#define PYROBATTLIB

#include <generallib.h>

// arrays to store the pins for access by index
const int enpins[4] = {P1_EN,P2_EN,P3_EN,P4_EN};
const int contpins[4] = {P1_CONT,P2_CONT,P3_CONT,P4_CONT};

//class to control all function of the pyro channels
class PYROCHANNEL{
volatile int _state = 0;
int identity;

int EN_PIN;
int CONT_PIN;
int fired = 0;

public:
    int continuity;
    uint32_t firedtime;
    uint32_t timeout = 1000;

    PYROCHANNEL(int _identifier);
    
    void fire();
    void checkfire();

    int getcont();
     int state();
};


PYROCHANNEL::PYROCHANNEL(int _identifier){
    identity = _identifier-1;
    EN_PIN = enpins[identity];
    CONT_PIN = contpins[identity];
    pinMode(EN_PIN,OUTPUT);
    digitalWrite(EN_PIN,LOW);
    pinMode(CONT_PIN,INPUT_PULLUP);
}

int PYROCHANNEL::state(){
    return _state;
}

//fires the channel, sets flag to turn it off with the interurptt
void PYROCHANNEL::fire(){
    if (_state == 0 && fired == 0)
    {
        Serial.printf("firing pyro %d\n",identity+1);
        firedtime = millis();
        
        digitalWrite(EN_PIN,HIGH);
        _state = 1;
        fired = 1;
    }
    return;
}

//check to see if the pyro has been on for long enough, if its past its timeout then turn it off
void PYROCHANNEL::checkfire(){
    if (_state >= 1 && millis()-firedtime > timeout)
    {
        //Serial.printf("disabling pyro %d\n",identity);
        digitalWrite(EN_PIN,LOW);
        _state = 0;
    }
    return;
}

//return the continuity status of the channel.
int PYROCHANNEL::getcont(){
    continuity = !digitalRead(CONT_PIN);
    return continuity;
}


#endif //PYROBATTLIB