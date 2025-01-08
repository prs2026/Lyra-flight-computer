/**************************************************************************/
/*!
    @file     Adafruit_ADXL375.h
    @author   Bryan Siepert and K. Townsend (Adafruit Industries)

    BSD license (see license.txt)

    This is a library for the Adafruit ADXL375 breakout board
    ----> https://www.adafruit.com

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    v1.0  - First release
*/
/**************************************************************************/
#ifndef _ADAFRUIT_SENSOR_ADXL375_H
#define _ADAFRUIT_SENSOR_ADXL375_H

#include "Arduino.h"
#include <Adafruit_ADXL343.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define ADXL375_ADDRESS (0x53) /**< Assumes ALT address pin low */
/*=========================================================================*/

/*=========================================================================
    REGISTERS ARE SAME AS ADXL343!
    -----------------------------------------------------------------------*/
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
#define ADXL375_MG2G_MULTIPLIER (0.049) /**< 49mg per lsb */
/*=========================================================================*/

/**
 * Driver for the Adafruit ADXL375 breakout.
 */
class Adafruit_ADXL375 : public Adafruit_ADXL343 {
public:
  Adafruit_ADXL375(int32_t sensorID);
  Adafruit_ADXL375(int32_t sensorID, TwoWire *wireBus);
  Adafruit_ADXL375(uint8_t cs, SPIClass *theSPI, int32_t sensorID = -1);
  Adafruit_ADXL375(uint8_t clock, uint8_t miso, uint8_t mosi, uint8_t cs,
                   int32_t sensorID = -1);

  bool begin(uint8_t i2caddr = ADXL375_ADDRESS);

  // these don't actually do anything
  void setRange(adxl34x_range_t range);
  adxl34x_range_t getRange(void);

  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
};

#endif
