#if !defined(LCDLIB)
#define LCDLIB
#include <Arduino.h>
#include <macros.h>

// KS0108 GLCD library initialization according to the following connection:
// KS0108_GLCD(DI, RW, E, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, CS1, CS2, RES);
KS0108_GLCD lcddr = KS0108_GLCD(LCDDI, LCDRW, LCDE, LCDDB0, LCDDB1, LCDDB2, LCDDB3, LCDDB4, LCDDB5, LCDDB6, LCDDB7, LCDCS1, LCDCS2, LCDRST);



class LCDDISPLAY
{
private:
    /* data */
public:
    LCDDISPLAY(){};

    int init();
    int drawtitlescreen();
    int drawtelemetryscreen(packets inpacket);

};

int LCDDISPLAY::init(){
    if ( lcddr.begin(KS0108_CS_ACTIVE_HIGH) == false ) {
        Serial.println( F("lcddr initialization failed!") );    // lack of RAM space
    }
    Serial.println("lcddr inited");
    lcddr.setTextColor(1);
    lcddr.setTextSize(1);
    lcddr.display();
    delay(500); // Pause for 2 seconds

    // Clear the buffer
    lcddr.clearDisplay();

    // Draw a single pixel in white
    lcddr.drawCircle(64,32,10,1);

    // Show the lcddr buffer on the screen. You MUST call lcddr() after
    // drawing commands to make them visible on screen!
    lcddr.display();
    


}

int LCDDISPLAY::drawtitlescreen(){
    lcddr.clearDisplay();
    lcddr.setCursor(30,25);
    lcddr.print("LERO RECIVER");
    
    lcddr.drawRect(10,10,110,40,1);
    lcddr.display();
    
}

int LCDDISPLAY::drawtelemetryscreen(packets inpacket){
    lcddr.clearDisplay();

    lcddr.setCursor(30,5);
    lcddr.print("Ground Idle");

    lcddr.setCursor(0,20);
    lcddr.print(" Altitude "); lcddr.println(inpacket.altitude);
    lcddr.print(" Speed "); lcddr.println(inpacket.verticalvel);
    lcddr.print(" Data Age "); lcddr.println(inpacket.verticalvel);

    
    // lcddr.drawRect(10,10,110,40,1);
    lcddr.display();
}





#endif // LCDLIB