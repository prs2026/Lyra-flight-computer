#include <Arduino.h>
#include <macros.h>
#include <radio.h>


RADIO radio;

void setup() {
  
  Serial.begin();
  Serial.print("init");
  pinMode(LEDINDICATION,OUTPUT);
  digitalWrite(LEDINDICATION,HIGH);
  delay(500);
  digitalWrite(LEDINDICATION,LOW);
  
  radio.init();
  Serial.println("radio good");
  Lora.request();
}

void loop() {

  if (Lora.available())
  {
    uint8_t message;
    message = Lora.read();
    Serial.printf("new message: %d",message);
  }
  
}