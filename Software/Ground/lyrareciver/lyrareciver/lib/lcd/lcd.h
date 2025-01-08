// #if !defined(LCDLIB3)
// #define LCDLIB3
// // core libs
// #include <Arduino.h>
// #include <macros.h>
// #include <TFT_eSPI.h>

// #define TFT_GREY 0x5AEB

// TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

// float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
// float sdeg=0, mdeg=0, hdeg=0;
// uint16_t osx=120, osy=120, omx=120, omy=120, ohx=120, ohy=120;  // Saved H, M, S x & y coords
// uint16_t x0=0, x1=0, yy0=0, yy1=0;   

// class LCDDISPLAY
// {
// private:
//     /* data */
// public:
//     LCDDISPLAY(/* args */);
//     bool init();
// };

// LCDDISPLAY::LCDDISPLAY(/* args */)
// {
// }

// bool LCDDISPLAY::init(){
//     Serial.println("init display");
//     digitalWrite(LCDCS,LOW);
//     digitalWrite(LCDWR,HIGH);
//     uint8_t recived = SPI.transfer(0x04);
//     digitalWrite(LCDCS,HIGH);
//     digitalWrite(LCDWR,LOW);
//     SPI.end();

//     tft.init();

//     tft.

//     tft.fillScreen(TFT_GREY);
//     tft.drawCircle(50,50,50,TFT_BLACK);
    
//     Serial.printf("recived: %d\n",recived);
//     Serial.println("out of lcd init");
//     return 1;
// }


// #endif // LCDLIB