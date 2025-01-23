#include <Arduino.h>
#include <macros.h>
#include <radio.h>
<<<<<<< Updated upstream
<<<<<<< Updated upstream
//#include <lcd.h>
#include <sdcard.h>


RADIO radio;
//LCDDISPLAY lcddisplay;
SDCARD sdcard;
=======
//#include <sdcard.h>


RADIO radio;
//SDCARD sdcard;
>>>>>>> Stashed changes
=======
//#include <sdcard.h>


RADIO radio;
//SDCARD sdcard;
>>>>>>> Stashed changes

telepacket currentstate;

uint32_t serialtime;
bool ready = false;

uint32_t dataage = 0;
uint32_t packettime;

void setup() {
  delay(3000);
  Serial.begin();
  Serial.print("init");
  pinMode(LEDINDICATION,OUTPUT);
  digitalWrite(LEDINDICATION,HIGH);
  delay(500);
  digitalWrite(LEDINDICATION,LOW);
  radio.init();
  //sdcard.init();
  Serial.println("radio good");
<<<<<<< Updated upstream
<<<<<<< Updated upstream
  
  //lcddisplay.init();

=======
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes
  ready = true;
}

void setup1(){
  while (ready == false)
  {
    delay(100);
  }
}

void loop() {

  if (millis() - serialtime > 100)
  {
    serialtime = millis();
    //Serial.printf(">Data age: %f\n",float((millis()-packettime))/1000);
  }
  
  if (digitalRead(BUTTON1) && digitalRead(BUTTON2) && digitalRead(BUTTON3))
  {
    Serial.println("sending packet");
    Lora.beginPacket();
    int j = 0;
    Lora.write('P');
    Lora.write('A');
    Lora.write('1');
    if (!Lora.endPacket())
    {
        Serial.println("packet send fail");
    }
    Serial.printf("error code :%d \n",Lora.getError());
    delay(1000);
  }
  
  
  //Serial.println("loop");
  
}

void loop1(){
  Lora.request();
  Lora.wait();
  if (Lora.available())
  {
    currentstate = radio.recivepacket();
    Serial.println("newpacket");
    Serial.printf(">State: %d\n",currentstate.r.state);
    Serial.printf(">Altitude: %f\n",float(currentstate.r.altitude)/10);
    Serial.printf(">Verticalvel: %f\n",float(currentstate.r.verticalvel)/100);
    Serial.printf(">RSSI: %f\n",radio.laspacketrssi);
    Serial.printf(">Accel X: %f\n",float(currentstate.r.accel.x)/100);
    Serial.printf(">Accel Y: %f\n",float(currentstate.r.accel.y)/100);
    Serial.printf(">Accel Z: %f\n",float(currentstate.r.accel.z)/100);
    Serial.printf(">Gyro X: %f\n",float(currentstate.r.gyro.x)/100);
    Serial.printf(">Gyro Y: %f\n",float(currentstate.r.gyro.y)/100);
    Serial.printf(">Gyro Z: %f\n",float(currentstate.r.gyro.z)/100);
    Serial.printf(">Pitch: %f\n",float(currentstate.r.orientationeuler.x)/100);
    Serial.printf(">Yaw: %f\n",float(currentstate.r.orientationeuler.y)/100);
    Serial.printf(">Roll: %f\n",float(currentstate.r.orientationeuler.z)/100);
    Serial.printf(">Uptime: %d\n",currentstate.r.uptime);
    packettime = millis();
    digitalWrite(LEDINDICATION,HIGH);
    delay(100);
    digitalWrite(LEDINDICATION,LOW);
    Serial.print("recivedpacket");
  }
}