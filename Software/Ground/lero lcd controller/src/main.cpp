#include <Arduino.h>
#include <MACROS.h>
#include "KS0108_GLCD.h"    // include KS0108 GLCD library
#include <lcdlib.h>
#include "SerialTransfer.h"


SerialTransfer myTransfer;

LCDDISPLAY display;

telepacket newpacket;


void setup(void) {

  pinMode(LED_BUILTIN,OUTPUT);

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(9600);



  Serial.println("init");\
  

  Serial1.begin(9600);
  myTransfer.begin(Serial1);

  display.init();

  display.drawtitlescreen();

  delay(2000);

  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.println("out of setup");
  display.drawtelemetryscreen();
}

void loop() {
  //display.display();
  if (myTransfer.available())
  {
    myTransfer.rxObj(newpacket.r);
    Serial.println(newpacket.r.altitude);
  }
  
  Serial1.write(123);
  delay(500);
  

}