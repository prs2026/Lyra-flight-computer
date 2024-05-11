/***************************************************
  This is our library for the   ST7789 Breakout and Shield
  ----> http://www..com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
   invests time and resources providing this open source code,
  please support  and open-source hardware by purchasing
  products from !

  Written by Limor Fried/Ladyada for  Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ST7789H_
#define _ST7789H_

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif
#include <GFX.h>
#include <avr/pgmspace.h>


#define ST7789_TFTWIDTH  240
#define ST7789_TFTHEIGHT 320

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_RDMODE  0x0A
#define ST7789_RDMADCTL  0x0B
#define ST7789_RDPIXFMT  0x0C
#define ST7789_RDIMGFMT  0x0A
#define ST7789_RDSELFDIAG  0x0F

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_GAMMASET 0x26
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29

#define ST7789_CASET   0x2A
#define ST7789_PASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_MADCTL  0x36
#define ST7789_PIXFMT  0x3A

#define ST7789_FRMCTR1 0xB1
#define ST7789_FRMCTR2 0xB2
#define ST7789_FRMCTR3 0xB3
#define ST7789_INVCTR  0xB4
#define ST7789_DFUNCTR 0xB6

#define ST7789_PWCTR1  0xC0
#define ST7789_PWCTR2  0xC1
#define ST7789_PWCTR3  0xC2
#define ST7789_PWCTR4  0xC3
#define ST7789_PWCTR5  0xC4
#define ST7789_VMCTR1  0xC5
#define ST7789_VMCTR2  0xC7

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

#define ST7789_GMCTRP1 0xE0
#define ST7789_GMCTRN1 0xE1
/*
#define ST7789_PWCTR6  0xFC

*/

// Color definitions
#define	ST7789_BLACK   0x0000
#define	ST7789_BLUE    0x001F
#define	ST7789_RED     0xF800
#define	ST7789_GREEN   0x07E0
#define ST7789_CYAN    0x07FF
#define ST7789_MAGENTA 0xF81F
#define ST7789_YELLOW  0xFFE0  
#define ST7789_WHITE   0xFFFF


class ST7789 : public GFX {

 public:

  ST7789(int8_t _CS, int8_t _DC, int8_t _MOSI, int8_t _SCLK,
		   int8_t _RST, int8_t _MISO);
  ST7789(int8_t _CS, int8_t _DC, int8_t _RST = -1);

  void     begin(void),
           setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1),
           pushColor(uint16_t color),
           fillScreen(uint16_t color),
           drawPixel(int16_t x, int16_t y, uint16_t color),
           drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
           drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
           fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
             uint16_t color),
           setRotation(uint8_t r),
           invertDisplay(boolean i);
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b);

  /* These are not for current use, 8-bit protocol only! */
  uint8_t  readdata(void),
    readcommand8(uint8_t reg, uint8_t index = 0);
  /*
  uint16_t readcommand16(uint8_t);
  uint32_t readcommand32(uint8_t);
  void     dummyclock(void);
  */  

  void     spiwrite(uint8_t),
    writecommand(uint8_t c),
    writedata(uint8_t d),
    commandList(uint8_t *addr);
  uint8_t  spiread(void);

 private:
  uint8_t  tabcolor;



  boolean  hwSPI;
#if defined (__AVR__) || defined(TEENSYDUINO)
  uint8_t mySPCR;
  volatile uint8_t *mosiport, *clkport, *dcport, *rsport, *csport;
  int8_t  _cs, _dc, _rst, _mosi, _miso, _sclk;
  uint8_t  mosipinmask, clkpinmask, cspinmask, dcpinmask;
#elif defined (__arm__)
    volatile RwReg *mosiport, *clkport, *dcport, *rsport, *csport;
    uint32_t  _cs, _dc, _rst, _mosi, _miso, _sclk;
    uint32_t  mosipinmask, clkpinmask, cspinmask, dcpinmask;
#endif
};

#endif
