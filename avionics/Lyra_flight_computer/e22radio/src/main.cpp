#include <Arduino.h>

bool mode = 0;

const int Mpin = 2;

void printBin(byte aByte) {
  for (int8_t aBit = 7; aBit >= 0; aBit--)
    Serial.write(bitRead(aByte, aBit) ? '1' : '0');
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  pinMode(Mpin,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  digitalWrite(Mpin,mode);
  Serial.begin(115200);
  while (!Serial) delay(100);
  Serial.println("\n\n init");

  // Serial1.end();
  // Serial1.setTX(0);
  // Serial1.setRX(1);
  Serial1.begin(9600);

}

void loop() {
  if (Serial.available())
  {
    uint8_t readbuf[20] = {};
    int j = 0;
    while (Serial.available() > 0)
    {
      readbuf[j] = Serial.read();
      //Serial.write(readbuf[j]);
      j++;
    }
    if (readbuf[0] == 'M')
    {
      mode = !mode;
      Serial.printf("switching modes, mode %d\n",mode);
      digitalWrite(Mpin,mode);
    }
    else
    {
      if (readbuf[0]==0xC1)
      {
        Serial.print("sending: ");
        Serial.printf("%x,%x,%x\n",readbuf[0],readbuf[1],readbuf[2]);
        Serial1.write(readbuf,3);
      }
      else
      {
        Serial.print("sending: ");
        Serial.printf("%x,%x,%x,%x\n",readbuf[0],readbuf[1],readbuf[2],readbuf[3]);
        Serial1.write(readbuf,4);
      }
      
      
    }
  }
  if (Serial1.available())
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
