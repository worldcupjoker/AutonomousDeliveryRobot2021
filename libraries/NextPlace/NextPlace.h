/*
 * File name: NextPlace.h
 * ----------------------
 * Library for creating a NextPlace object.
 * https://www.arduino.cc/en/Hacking/libraryTutorial
 * https://www.learncpp.com/cpp-tutorial/class-code-and-header-files/
 * https://docs.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-160
 */
#ifndef NextPlace_h
#define NextPlace_h

#include "Arduino.h"

class NextPlace {
  public:
    NextPlace(String location, double mapData[]);
    double getX();
    double getY();
    String toString();

  private:
    double x, y, blockSize;
    int blockNumX, blockNumY;
    String _location;
};

#endif
