#include <Arduino.h>
#include <MACROS.h>
#include "KS0108_GLCD.h"    // include KS0108 GLCD library


// KS0108 GLCD library initialization according to the following connection:
// KS0108_GLCD(DI, RW, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, CS1, CS2, RES);
KS0108_GLCD display = KS0108_GLCD(LCDDI, LCDRW, LCDE, LCDDB0, LCDDB1, LCDDB2, LCDDB3, LCDDB4, LCDDB5, LCDDB6, LCDDB7, LCDCS1, LCDCS2, LCDRST);


void setup(void) {

  pinMode(LED_BUILTIN,OUTPUT);

  digitalWrite(LED_BUILTIN,HIGH);
  Serial.begin(9600);
  while (!Serial);
  delay(500);
  Serial.println("init");

  if ( display.begin(KS0108_CS_ACTIVE_HIGH) == false ) {
    Serial.println( F("display initialization failed!") );    // lack of RAM space
  }
  Serial.println("display inited");
  display.display();
  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawCircle(64,32,10,1);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();

  display.invertDisplay(true);
  
  display.display();

  delay(2000);

  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.println("out of setup");
}

void loop() {
  //display.display();
  delay(1000);

}