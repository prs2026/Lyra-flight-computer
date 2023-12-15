#include <Arduino.h>
#include <E220.h>

Stream &radioserial = (Stream &)Serial1;
                    // m0m1aux
E220 ebyte(&radioserial,2,3,4);


bool mode = 0;

const int Mpin = 2;

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
  Serial.printf("error: %d new address: %x \n",ebyte.setAddress(0xffff,true),ebyte.getAddress());

  Serial.printf("error: %d new power: %d \n",ebyte.setPower(Power_21,true),ebyte.getPower());

  Serial.printf("error: %d new channel: %d \n",ebyte.setChannel(68,true),ebyte.getChannel());
  
  ebyte.printBoardParameters();
}

void loop() {
  if (Serial.available())
  {
    char readbuf = Serial.read();
    Serial.printf("recived: %d,",readbuf);
    uint8_t sendbyte = Serial.read();
    Serial.printf("%d\n",sendbyte);
    switch (readbuf)
    {
    case 's':
      
      Serial.printf("sending: %d \n",sendbyte);
      Serial1.print(sendbyte);
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
