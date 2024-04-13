#include <Arduino.h>
#include <macros.h>
#include <radio.h>


RADIO radio;

void setup() {
  delay(3000);
  Serial.begin();
  Serial.print("init");
  pinMode(LEDINDICATION,OUTPUT);
  digitalWrite(LEDINDICATION,HIGH);
  delay(500);
  digitalWrite(LEDINDICATION,LOW);
  
  radio.init();
  Serial.println("radio good");
  Lora.request(SX126X_RX_CONTINUOUS);
}

void loop() {

  if (Lora.available())
  {
    uint8_t message;
    message = Lora.read();
    Serial.printf("new message: %d",message);
    Lora.request();
  }
  
}