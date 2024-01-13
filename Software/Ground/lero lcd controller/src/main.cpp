#include <Arduino.h>
#include <MACROS.h>
#include "KS0108_GLCD.h"    // include KS0108 GLCD library
#include <lcdlib.h>


LCDDISPLAY display;


void setup(void) {

  pinMode(LED_BUILTIN,OUTPUT);

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(9600);



  Serial.println("init");\
  

  Serial1.begin(9600);

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
  if (Serial1.available());
  {
    Serial.print("new messsage: ");
    Serial.println(Serial1.read());
  }
  

}