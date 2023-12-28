#include <pyrobatt.h>


PYROCHANNEL::PYROCHANNEL(int _identifier){
    identity = _identifer;
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


void PYROCHANNEL::getcont(){
    contuinty = digitalRead(CONT_PIN);
    return;
}

int PYROCHANNEL::getcont(){
    contuinty = digitalRead(CONT_PIN);
    return contuinty;
}