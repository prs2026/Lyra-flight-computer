#include <Arduino.h>
#include <E220.h>

Stream &radioserial = (Stream &)Serial1;
                    // m0m1aux
E220 ebyte(&radioserial,3,2,4);

uint32_t ledtime = 0;
bool ledstate = true;

void printBin(byte aByte) {
  for (int8_t aBit = 7; aBit >= 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.begin(115200);
  while (!Serial) delay(100);
  Serial.println("\n\n init");
  Serial1.begin(9600);
  while (!ebyte.init())
  {
    Serial.println("radio init fail");
  }
  ebyte.setAddress(0xffff,true);
  ebyte.setPower(Power_21,true);
  ebyte.setChannel(68,true);
  ebyte.setSubPacketSize(SPS_64,true);
  ebyte.setAirDataRate(UDR_1200,true);
  ebyte.setEncryptionKey(0,true);
  ebyte.setLBT(true,true);
  ebyte.printBoardParameters();
  // Serial.println("testintinting");

  // Serial.end();
  // delay(2000);
  // Serial1.write(0x34);
  // delay(500);
  // Serial.begin();
  // Serial.println("done");
  // delay(3000);
  // Serial1.write(0x54);
}

void loop() {
  if (millis()-ledtime > 1000)
  {
    ledtime = millis();
    digitalWrite(LED_BUILTIN,ledstate);
    ledstate = !ledstate;
  }
  
  if (Serial.available())
  {
    char readbuf = Serial.read();
    Serial.printf("recived: %d,",readbuf);
    uint8_t sendbyte = Serial.read();
    Serial.printf("%d\n",sendbyte);
    switch (readbuf)
    {
    case 0xc6:
      
      Serial.printf("sending: %d \n",sendbyte);
      Serial1.write(sendbyte);
      break;
    
    default:
      break;
    }
    Serial.println("out of sending");
  }
  if (Serial1.available() > 0)
  {
    Serial.println("new message: ");
    while (Serial1.available() > 0)
    {

      uint8_t buf = Serial1.read();
      Serial.printf("0x%x, ",buf);
      printBin(buf);
      Serial.printf(", %d\n",buf);
      delay(50);
    }
    
  }

  
}