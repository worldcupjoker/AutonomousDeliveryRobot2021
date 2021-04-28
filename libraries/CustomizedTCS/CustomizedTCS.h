/*
 * File name: CustomizedTCS.h
 * ----------------------
 * Library for creating a customized TCS object that uses software i2c communication,
 * using library: Adafruit_TCS34725softi2c.h (https://github.com/Fire7/Adafruit_TCS34725_SoftI2C)
 * The library misses the maping from raw data to RGB data for led strips.
 * This customized library fixes the mapping.
 * 
 * To use this library, create an object: CustomizedTCS tcs = CustomizedTCS(SDApin, SCLpin);
 * Initialize the sensor: tcs.start();
 * uint16_t lux = tcs.getRGB(&redValue, &greenValue, &blueValue);
 * Then you are all set.
 */
#ifndef CustomizedTCS_h
#define CustomizedTCS_h

#include "Arduino.h"
#include "Adafruit_TCS34725softi2c.h"

class CustomizedTCS {
  public:
    CustomizedTCS(int pinSDA, int pinSCL);
    uint16_t getRGB(uint8_t *r, uint8_t *g, uint8_t *b);
    void start();

  private:
    Adafruit_TCS34725softi2c tcs;
    uint8_t gammaTable[256];
};

#endif