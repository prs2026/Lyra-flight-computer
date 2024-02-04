#include <Arduino.h>
#include <MACROS.h>
#include "KS0108_GLCD.h"    // include KS0108 GLCD library
#include <lcdlib.h>
#include "SerialTransfer.h"
#include <stdlib.h>

SerialTransfer myTransfer;

LCDDISPLAY display;


uint32_t buttonchecktime = 0;

datapacket newpacket;

void checkbuttons(){
  int buttonstatus[5] = {};

  buttonstatus[0] = digitalRead(BUTTON1);
  buttonstatus[1] = digitalRead(BUTTON2);
  buttonstatus[2] = digitalRead(BUTTON3);
  buttonstatus[3] = digitalRead(BUTTON4);
  buttonstatus[4] = digitalRead(BUTTON5);
  int j = 0;
  Serial.println("\nbuttons: ");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(buttonstatus[j]);
    Serial.print(" ");
  }
  Serial.println();
  

}




void setup(void) {

  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2,INPUT);
  pinMode(BUTTON3,INPUT);
  pinMode(BUTTON4,INPUT);
  pinMode(BUTTON5,INPUT);

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
    int32_t result[8] = {};
    uint32_t starttime = millis();
    int32_t j = 0;
    while (Serial1.available() && millis()-starttime < 400)
    {
      char buf[15] = "";
      Serial1.readBytesUntil(',',buf,15);
      //Serial.print(buf);
      //Serial.print(" ");
      delay(10);
      if (buf != "A" || buf != "D")
      {
        int32_t num = int32_t(strtol(buf,0,10));
        //Serial.print(num);
        //Serial.print(" ");
        result[j] = num;
        j++;
      }
      
    }
    // Serial.println("");

    newpacket.altitude = result[1];
    newpacket.verticalvel = result[2];
    newpacket.dataage = result[4];
    newpacket.state = result[3];
    newpacket.status = result[5];

    if (newpacket.altitude > newpacket.maxalt)
    {
      newpacket.maxalt = newpacket.altitude;
    }
    
  }

  if (millis() - buttonchecktime > 250)
  {
    checkbuttons();
    buttonchecktime = millis();
  }
  
  display.drawtelemetryscreen(newpacket);
}