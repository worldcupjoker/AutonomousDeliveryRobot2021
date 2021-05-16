// CustomizedTCS.cpp
#include "CustomizedTCS.h"
#include "Arduino.h"

/* CustomizedTCS constructor */
CustomizedTCS::CustomizedTCS(int pinSDA, int pinSCL) {
  tcs = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_101MS, TCS34725_GAIN_4X, pinSDA, pinSCL);

  // Initialize the gamma table
  for (int i = 0; i < 256; i++) {
    float x = i;
    x /= 255;
    x = x * x * sqrt(x); // powf(x, 2.5) does not work in this loop. Don't know why.
    x *= 255;
    gammaTable[i] = x;
  }
}

/* CustomizedTCS member functions */

/* Initialize the sensor. Call this function before doing anything else. */
void CustomizedTCS::start() {
  tcs.begin();
}

/* Get RGB values */
uint16_t CustomizedTCS::getRGB (uint8_t *r, uint8_t *g, uint8_t *b) {
  uint16_t red, green, blue, clear;
  tcs.getRawData(&red, &green, &blue, &clear);
  *r = gammaTable[map(red, 0, clear, 0, 255)];
  *g = gammaTable[map(green, 0, clear, 0, 255)];
  *b = gammaTable[map(blue, 0, clear, 0, 255)];
  return tcs.calculateLux(red,green,blue);
}
