/*************************************************** 
  www.buydisplay.com
 ****************************************************/

#include "ST7789.h"
#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

// Constructor when using software SPI.  All output pins are configurable.
ST7789::ST7789(int8_t cs, int8_t dc, int8_t mosi,
				   int8_t sclk, int8_t rst, int8_t miso) : GFX(ST7789_TFTWIDTH, ST7789_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ST7789::ST7789(int8_t cs, int8_t dc, int8_t rst) : GFX(ST7789_TFTWIDTH, ST7789_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _sclk = 0;
}

void ST7789::spiwrite(uint8_t c) {

  //Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

  if (hwSPI) {
#if defined (__AVR__)
      uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
    SPCR = backupSPCR;
#elif defined(TEENSYDUINO)
    SPI.transfer(c);
#elif defined (__arm__)
    SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.transfer(c);
#endif
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	//digitalWrite(_mosi, HIGH); 
	*mosiport |=  mosipinmask;
      } else {
	//digitalWrite(_mosi, LOW); 
	*mosiport &= ~mosipinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }
  }
}


void ST7789::writecommand(uint8_t c) {
  *dcport &=  ~dcpinmask;
  //digitalWrite(_dc, LOW);
  //*clkport &= ~clkpinmask; // clkport is a NULL pointer when hwSPI==true
  //digitalWrite(_sclk, LOW);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  spiwrite(c);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void ST7789::writedata(uint8_t c) {
  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  //*clkport &= ~clkpinmask; // clkport is a NULL pointer when hwSPI==true
  //digitalWrite(_sclk, LOW);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);
  
  spiwrite(c);

  //digitalWrite(_cs, HIGH);
  *csport |= cspinmask;
} 

// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
#ifdef SPI_HAS_TRANSACTION
static inline void spi_begin(void) __attribute__((always_inline));
static inline void spi_begin(void) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
}
static inline void spi_end(void) __attribute__((always_inline));
static inline void spi_end(void) {
  SPI.endTransaction();
}
#else
#define spi_begin()
#define spi_end()
#endif

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ST7789::commandList(uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


void ST7789::begin(void) {
  if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);

  if(hwSPI) { // Using hardware SPI
#if defined (__AVR__)
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    mySPCR = SPCR;
#elif defined(TEENSYDUINO)
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
#elif defined (__arm__)
      SPI.begin();
      SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
      SPI.setBitOrder(MSBFIRST);
      SPI.setDataMode(SPI_MODE0);
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
    mosipinmask = digitalPinToBitMask(_mosi);
    *clkport   &= ~clkpinmask;
    *mosiport  &= ~mosipinmask;
  }

  // toggle RST low to reset
  if (_rst > 0) {
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(150);
  }

  /*
  uint8_t x = readcommand8(ST7789_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ST7789_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ST7789_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ST7789_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ST7789_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
*/
  //if(cmdList) commandList(cmdList);
  
  if (hwSPI) spi_begin();



	//----display and color format setting----------// 
 	writecommand(0x36); 
	writedata(0x00); 
 

    writecommand(0x3A);
	writedata(0x05);

 	writecommand(0xB2);
	writedata(0x0C);
	writedata(0x0C);
	writedata(0x00);
	writedata(0x33);
	writedata(0x33); 

 	writecommand(0xB7); 
	writedata(0x35);  

 	writecommand(0xBB);
	writedata(0x15);

 	writecommand(0xC0);
	writedata(0x2C);

 	writecommand(0xC2);
	writedata(0x01);

 	writecommand(0xC3);
	writedata(0x0B);   

 	writecommand(0xC4);
	writedata(0x20);  

 	writecommand(0xC6); 
	writedata(0x0F);    

 	writecommand(0xD0); 
	writedata(0xA4);
	writedata(0xA1);

 	writecommand(0x21); 
 	writecommand(0xE0);
	writedata(0xF0);
	writedata(0x00);
	writedata(0x04);
	writedata(0x04);
	writedata(0x04);
	writedata(0x05);
	writedata(0x29);
	writedata(0x33);
	writedata(0x3E);
	writedata(0x38);
	writedata(0x12);
	writedata(0x12);
	writedata(0x28);
	writedata(0x30);

 	writecommand(0xE1);
	writedata(0xF0);
	writedata(0x07);
	writedata(0x0A);
	writedata(0x0D);
	writedata(0x0B);
	writedata(0x07);
	writedata(0x28);
	writedata(0x33);
	writedata(0x3E);
	writedata(0x36);
	writedata(0x14);
	writedata(0x14);
	writedata(0x29);
	writedata(0x32);


 	

  writecommand(ST7789_SLPOUT);    //Exit Sleep 
  if (hwSPI) spi_end();
  delay(120); 		
  if (hwSPI) spi_begin();
  writecommand(ST7789_DISPON);    //Display on 
  if (hwSPI) spi_end();

}


void ST7789::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writecommand(ST7789_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART 
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ST7789_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ST7789_RAMWR); // write to RAM
}


void ST7789::pushColor(uint16_t color) {
  if (hwSPI) spi_begin();
  //digitalWrite(_dc, HIGH);
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
  if (hwSPI) spi_end();
}

void ST7789::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  if (hwSPI) spi_begin();
  setAddrWindow(x,y,x+1,y+1);

  //digitalWrite(_dc, HIGH);
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
  if (hwSPI) spi_end();
}


void ST7789::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
    h = _height-y;

  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
  if (hwSPI) spi_end();
}


void ST7789::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
  //digitalWrite(_dc, HIGH);
  //digitalWrite(_cs, LOW);
  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
  if (hwSPI) spi_end();
}

void ST7789::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void ST7789::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  //digitalWrite(_cs, HIGH);
  *csport |= cspinmask;
  if (hwSPI) spi_end();
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t ST7789::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ST7789::setRotation(uint8_t m) {

  if (hwSPI) spi_begin();
  writecommand(ST7789_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(MADCTL_MX | MADCTL_BGR);
     _width  = ST7789_TFTWIDTH;
     _height = ST7789_TFTHEIGHT;
     break;
   case 1:
     writedata(MADCTL_MV | MADCTL_BGR);
     _width  = ST7789_TFTHEIGHT;
     _height = ST7789_TFTWIDTH;
     break;
  case 2:
    writedata(MADCTL_MY | MADCTL_BGR);
     _width  = ST7789_TFTWIDTH;
     _height = ST7789_TFTHEIGHT;
    break;
   case 3:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     _width  = ST7789_TFTHEIGHT;
     _height = ST7789_TFTWIDTH;
     break;
  }
  if (hwSPI) spi_end();
}


void ST7789::invertDisplay(boolean i) {
  if (hwSPI) spi_begin();
  writecommand(i ? ST7789_INVON : ST7789_INVOFF);
  if (hwSPI) spi_end();
}


////////// stuff not actively being used, but kept for posterity


uint8_t ST7789::spiread(void) {
  uint8_t r = 0;

  if (hwSPI) {
#if defined (__AVR__)
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = 0x00;
    while(!(SPSR & _BV(SPIF)));
    r = SPDR;
    SPCR = backupSPCR;
#elif defined(TEENSYDUINO)
    r = SPI.transfer(0x00);
#elif defined (__arm__)
    SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    r = SPI.transfer(0x00);
#endif
  } else {

    for (uint8_t i=0; i<8; i++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_sclk, HIGH);
      r <<= 1;
      if (digitalRead(_miso))
	r |= 0x1;
    }
  }
  //Serial.print("read: 0x"); Serial.print(r, HEX);
  
  return r;
}

 uint8_t ST7789::readdata(void) {
   digitalWrite(_dc, HIGH);
   digitalWrite(_cs, LOW);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   
   return r;
}
 

uint8_t ST7789::readcommand8(uint8_t c, uint8_t index) {
   if (hwSPI) spi_begin();
   digitalWrite(_dc, LOW); // command
   digitalWrite(_cs, LOW);
   spiwrite(0xD9);  // woo sekret command?
   digitalWrite(_dc, HIGH); // data
   spiwrite(0x10 + index);
   digitalWrite(_cs, HIGH);

   digitalWrite(_dc, LOW);
   digitalWrite(_sclk, LOW);
   digitalWrite(_cs, LOW);
   spiwrite(c);
 
   digitalWrite(_dc, HIGH);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   if (hwSPI) spi_end();
   return r;
}


 
/*

 uint16_t ST7789::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t ST7789::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
