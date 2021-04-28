// NextPlace.cpp
#include "NextPlace.h"
#include "Arduino.h"

/* NextPlace constructor */
NextPlace::NextPlace(String location, double mapData[]) {
  blockSize = mapData[0] / mapData[2];
  blockNumY = location[0] - 'A' + 1;
  blockNumX = location[1] - '1' + 1;
  _location = location;
  //Serial.println(location);
  //Serial.println(blockNumY);
  //Serial.println(blockNumX);
}

/* NextPlace member functions */
/* Get x coordinate. */
double NextPlace::getX() {
  x = blockNumX * blockSize - blockSize / 2;
  return x;
}

/* Get y coordinate. */
double NextPlace::getY() {
  y = blockNumY * blockSize - blockSize / 2;
  return y;
}

/* Get block address. */
String NextPlace::toString() {
  return _location;
}