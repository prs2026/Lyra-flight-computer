/**************************************************************************/
/*!
    @file     Adafruit_ADXL375.cpp
    @author   Bryan Siepert and K.Townsend (Adafruit Industries)

    BSD License (see license.txt)

    The ADXL375 is a digital accelerometer with 13-bit resolution, capable
    of measuring +/-200g.  This driver communicates using I2C or SPI

    This is a library for the Adafruit ADXL375 breakout
    ----> https://www.adafruit.com/

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    v1.0 - First release
*/
/**************************************************************************/

#include "Adafruit_ADXL375.h"

/**************************************************************************/
/*!
 *   @brief  Instantiates a new ADXL375 class
 *
 *   @param sensorID  An optional ID # so you can track this sensor, it will
 *                    tag sensorEvents you create.
 */
/**************************************************************************/
Adafruit_ADXL375::Adafruit_ADXL375(int32_t sensorID)
    : Adafruit_ADXL343(sensorID) {}

/**************************************************************************/
/*!
 *   @brief  Instantiates a new ADXL375 class
 *
 *   @param sensorID  An optional ID # so you can track this sensor, it will
 *                    tag sensorEvents you create.
 *   @param wireBus   TwoWire instance to use for I2C communication.
 */
/**************************************************************************/
Adafruit_ADXL375::Adafruit_ADXL375(int32_t sensorID, TwoWire *wireBus)
    : Adafruit_ADXL343(sensorID, wireBus) {}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL375 class in hardware SPI mode
    @param cs The CS/SSEL pin
    @param theSPI SPIClass instance to use for SPI communication.
    @param sensorID An optional ID # so you can track this sensor, it will tag
           sensorEvents you create.
*/
/**************************************************************************/
Adafruit_ADXL375::Adafruit_ADXL375(uint8_t cs, SPIClass *theSPI,
                                   int32_t sensorID)
    : Adafruit_ADXL343(cs, theSPI, sensorID) {}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADXL375 class in software SPI mode

    @param clock The SCK pin
    @param miso The MISO pin
    @param mosi The MOSI pin
    @param cs The CS/SSEL pin
    @param sensorID An optional ID # so you can track this sensor, it will tag
           sensoorEvents you create.
*/
/**************************************************************************/
Adafruit_ADXL375::Adafruit_ADXL375(uint8_t clock, uint8_t miso, uint8_t mosi,
                                   uint8_t cs, int32_t sensorID)
    : Adafruit_ADXL343(clock, miso, mosi, cs, sensorID) {}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @param  i2caddr The 7-bit I2C address to find the ADXL on
    @return True if the sensor was successfully initialised.
*/
/**************************************************************************/
bool Adafruit_ADXL375::begin(uint8_t i2caddr) {

  if (_wire) {
    if (i2c_dev) {
      delete i2c_dev; // remove old interface
    }
    i2c_dev = new Adafruit_I2CDevice(i2caddr, _wire);

    if (!i2c_dev->begin()) {
      return false;
    }

  } else {
    i2c_dev = NULL;

    if (spi_dev) {
      delete spi_dev; // remove old interface
    }
    if (_spi) {
      // hardware spi
      spi_dev = new Adafruit_SPIDevice(_cs,
                                       1000000,               // frequency
                                       SPI_BITORDER_MSBFIRST, // bit order
                                       SPI_MODE3,             // data mode
                                       _spi);                 // hardware SPI
    } else {
      // software spi
      spi_dev = new Adafruit_SPIDevice(_cs, _clk, _di, _do,
                                       1000000,               // frequency
                                       SPI_BITORDER_MSBFIRST, // bit order
                                       SPI_MODE3);            // data mode
    }
    if (!spi_dev->begin()) {
      return false;
    }
  }

  /* Check connection */
  uint8_t deviceid = getDeviceID();
  if (deviceid != 0xE5) {
    /* No ADXL375 detected ... return false */
    return false;
  }

  // Default tap detection level (2G, 31.25ms duration, single tap only)
  // If only the single tap function is in use, the single tap interrupt
  // is triggered when the acceleration goes below the threshold, as
  // long as DUR has not been exceeded.
  writeRegister(ADXL3XX_REG_INT_ENABLE, 0);  // Disable interrupts to start
  writeRegister(ADXL3XX_REG_THRESH_TAP, 20); // 62.5 mg/LSB (so 0xFF = 16 g)
  writeRegister(ADXL3XX_REG_DUR, 50);        // Max tap duration, 625 µs/LSB
  writeRegister(ADXL3XX_REG_LATENT,
                0); // Tap latency, 1.25 ms/LSB, 0=no double tap
  writeRegister(ADXL3XX_REG_WINDOW,
                0); // Waiting period,  1.25 ms/LSB, 0=no double tap
  writeRegister(ADXL3XX_REG_TAP_AXES, 0x7); // Enable the XYZ axis for tap

  // Enable measurements
  writeRegister(ADXL3XX_REG_POWER_CTL, 0x08);

  // Force full range (fixes issue with DATA_FORMAT register's reset value)
  // Per datasheet, needs to be D4=0, D3=D1=D0=1
  writeRegister(ADXL3XX_REG_DATA_FORMAT, 0b00001011);

  return true;
}

/**************************************************************************/
/*!
    @brief Sets the g range for the accelerometer, cannot be changed!

    @param range Unused range argument
*/
/**************************************************************************/
void Adafruit_ADXL375::setRange(adxl34x_range_t range) { return; }

/**************************************************************************/
/*!
    @brief  Sets the g range for the accelerometer

    @return Always returns 0
*/
/**************************************************************************/
adxl34x_range_t Adafruit_ADXL375::getRange(void) { return (adxl34x_range_t)0; }

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event

    @param event Pointer to the sensors_event_t placeholder

    @return True of the read request was successful.
*/
/**************************************************************************/
bool Adafruit_ADXL375::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_ACCELEROMETER;
  event->timestamp = millis();
  event->acceleration.x =
      getX() * ADXL375_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.y =
      getY() * ADXL375_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  event->acceleration.z =
      getZ() * ADXL375_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data

    @param sensor Pointer to the sensor_t placeholder.
*/
/**************************************************************************/
void Adafruit_ADXL375::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "ADXL375", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_ACCELEROMETER;
  sensor->min_delay = 0;
  sensor->min_value = -1961.33F; /*  -200g  */
  sensor->max_value = 1961.33F;  /* +200g  */
  sensor->resolution = 0.4805F;  /*  49mg */
}
