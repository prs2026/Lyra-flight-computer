#include <Arduino.h>
#include <MACROS.h>
#include "KS0108_GLCD.h"    // include KS0108 GLCD library
#include <lcdlib.h>
#include "SerialTransfer.h"


SerialTransfer myTransfer;

LCDDISPLAY display;




packets newpacket;


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
  if (Serial1.available() && Serial1.read() == 'A')
  {
    delay(100);
    char buf [10];
    uint32_t numbuf[17];
    int counter = 0;
    while (Serial1.available())
    {
      byte m = Serial1.readBytesUntil(',', buf, 10); //receive/store every data item separtaed by comma
      buf[m] = '\0';  //nul charcater
      numbuf[counter] = atoi(buf);  //retrieve original decimal number
      Serial.println(numbuf[counter]);
      counter++;
    }
    
    newpacket.orientationeuler.x = numbuf[1];
    newpacket.orientationeuler.y = numbuf[2];
    newpacket.orientationeuler.z = numbuf[3];

    newpacket.accel.x = numbuf[4];
    newpacket.accel.y = numbuf[5];
    newpacket.accel.z = numbuf[6];
    
    newpacket.gyro.x = numbuf[7];
    newpacket.gyro.y = numbuf[8];
    newpacket.gyro.z = numbuf[9];
    
    newpacket.altitude = numbuf[10];
    newpacket.verticalvel = numbuf[11];
    newpacket.uptime = numbuf[12];


    newpacket.errorflagmp = numbuf[13];
    newpacket.errorflagnav = numbuf[14];

    newpacket.state = numbuf[15];
    newpacket.dataage = numbuf[16];
    int j = 0;
    for (int i = 0; i < 17; i++)
    {
      Serial.print(j); Serial.print(" ");
      Serial.println(numbuf[j]);
      j++;
    }
    


    Serial.println(buf);
  }
  display.drawtelemetryscreen(newpacket);
}