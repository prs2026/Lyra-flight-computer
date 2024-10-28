#include <Arduino.h>
#include <BASIC.h>
#include <MAINCORE.h>
#include <SENSECORE.h>

MAINCORE MCORE;
SENSECORE SCORE;

void setup() {
  setled(0);
  delay(3000);
  Serial.begin(115200);
  setuppins();
  beep(200,4000);
  setled(1);
  Serial.println("Mcore init");
  MCORE.status = 1;
}

void setup1() {
  while (MCORE.status != 1)
  {
    delay(100);
  }

  SCORE.I2Cinit(1);
  Serial.println("Score init");
}

void loop() {
  
}

void loop1() {
  
}