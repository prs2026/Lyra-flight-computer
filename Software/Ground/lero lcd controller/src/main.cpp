#include <Arduino.h>
#include <MACROS.h>
#include "KS0108_GLCD.h"    // include KS0108 GLCD library
#include <lcdlib.h>
#include "SerialTransfer.h"
#include <stdlib.h>

SerialTransfer myTransfer;

LCDDISPLAY display;




datapacket newpacket;


void setup(void) {

  pinMode(LED_BUILTIN,OUTPUT);

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(9600);

  display.init();

  display.drawtitlescreen();

  Serial.println("init");\
  

  Serial1.begin(9600);
  myTransfer.begin(Serial1);

  digitalWrite(LED_BUILTIN,LOW);
  delay(100);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.println("out of setup");
  display.drawtelemetryscreen(newpacket);
}

void loop() {
  //display.display();
  // if (Serial1.available() && Serial1.peek() == 'A')
  // {
  //   delay(100);
  //   Serial.println("new packet");
  //   char buf [30];
  //   int32_t numbuf[20];
  //   int counter = 0;
  //   uint32_t packetstarttime = millis();
  //   while (Serial1.available() && millis()-packetstarttime < 250)
  //   {
  //     byte m = Serial1.readBytesUntil(',', buf, 30); //receive/store every data item separtaed by comma
  //     buf[m] = '\0';  //nul charcater
  //     Serial.print(buf);
  //     Serial.print(" ");
  //     numbuf[counter] = atoi(buf);  //retrieve original decimal number
  //     //Serial.println(numbuf[counter]);
  //     counter++;
  //     delay(1);
  //   }

  //   Serial.println("");
  if (Serial1.available())
  {
    //Serial.println("trasmission");
    int32_t result[6] = {};
    uint32_t starttime = millis();
    int32_t j = 0;
    while (Serial1.available() && millis()-starttime < 400)
    {
      char buf[15] = "";
      Serial1.readBytesUntil(',',buf,15);
      Serial.print(buf);
      Serial.print(" ");
      delay(10);
      if (buf != "A" || buf != "D")
      {
        int32_t num = int32_t(strtol(buf,0,10));
        Serial.print(num);
        Serial.print(" ");
        result[j] = num;
        j++;
      }
      
    }
    // Serial.println("");

    newpacket.altitude = result[1];
    newpacket.verticalvel = result[2];
    newpacket.dataage = result[4];
    newpacket.state = result[3];

    if (newpacket.altitude > newpacket.maxalt)
    {
      newpacket.maxalt = newpacket.altitude;
    }
    
  }

  
  display.drawtelemetryscreen(newpacket);
}