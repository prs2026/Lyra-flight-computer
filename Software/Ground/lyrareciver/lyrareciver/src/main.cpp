#include <Arduino.h>
#include <macros.h>
#include <radio.h>


RADIO radio;

void setup() {
  pinMode(LEDINDICATION,OUTPUT);
  radio.init();
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