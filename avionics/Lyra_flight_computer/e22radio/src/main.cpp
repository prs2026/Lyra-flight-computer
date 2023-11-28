#include <Arduino.h>
#include <LoRa_E220.h>

LoRa_E220 radio(&Serial1);


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(115200);
  while (!Serial) delay(100);
  Serial.println("\n\n init");

  // Serial1.end();
  Serial1.setTX(0);
  Serial1.setRX(1);
  // Serial1.begin(9600);
  Serial.println("initiing radio");
  int error = radio.begin();

  Serial.printf("radio status: %d\n",error);
}

void loop() {
  // put your main code here, to run repeatedly:
}
