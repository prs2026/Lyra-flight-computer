#include <Arduino.h>
#include <MACROS.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <SD.h>

U8G2_ST7920_128X64_1_SW_SPI u8g2(0, LCDE, LCDRW, LCDDI);

void setup(void) {

  pinMode(LCDDI,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(115200);
  while (!Serial);
  delay(500);
  Serial.println("init");

  // SPI.setTX(MOSISD);
  // SPI.setRX(MISOSD);
  // SPI.setSCK(SCKSD);
  SPI.begin();
  // SPI.end();

  Serial.println("SPI init");

  if (!SD.begin(CS_SD,SPI))
  {
    Serial.println("SD init fail");
  }

  File dataFile = SD.open("/datalog.txt", FILE_WRITE);
  if (!dataFile)
  {
    Serial.println("SD file init fail");
  }
  dataFile.print("testing");
  dataFile.close();
  

  
  int error = u8g2.begin();
  Serial.printf("lcd error: %d\n",error);

  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.println("out of setup");
}

void loop() {
  // u8g2.firstPage();
  // do {
  //   u8g2.setFont(u8g2_font_ncenB14_tr);
  //   u8g2.drawStr(0,24,"Hello World!");
  // } while ( u8g2.nextPage() );
}