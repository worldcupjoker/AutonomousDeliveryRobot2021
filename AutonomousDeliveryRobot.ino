/*
 * Copyright, 2021, (Ted) Zhongkun Zhan, all rights reserved
 * File: DeliveryRobot.ino
 * ----------------------
 * Name: Omar W. Hussaein, Santiago Guevara Ocana, (Ted) Zhongkun Zhan
 * This program will drive a robot to pick up food from differenct places
 * and deliver them to the customer's location.
 */

/* Customized library */
#include "NextPlace.h"
#include "CustomizedTCS.h"

/* Downloaded library */
#include <Wire.h>
#include <afstandssensor.h>
#include <VL53L1X.h>
#include <SPI.h>
#include "SevSeg.h"

VL53L1X distanceSensor;
VL53L1X myLeftSensor;
VL53L1X myLeftSensor2;
SevSeg sevSeg; 

/* Instance variables */
/* Type in the destinations here.*/
const String locations[] = {"F3", "E5", "G4"};
const int PLACES_NUMBER = sizeof(locations) / sizeof(locations[0]);

/* Map constants */
const double MAP_WIDTH = 7 / 3.2808;//7 / 3.2808;
const double BLOCK_NUMBERS = 7;//7;
const double MAP_DATA[] = {MAP_WIDTH, MAP_WIDTH, BLOCK_NUMBERS};
const double WALL_GAP =0.5 / 3.2808;// 0.5 / 3.2808;

/* Car constants */
const double COLORSENSOR_POS_VER = 0.165;
const double COLORSENSOR_POS_HOR = 0.055;

/* Pin constants */
const int MOTOR_RIGHT_IN3 = 24; // Forward
const int MOTOR_RIGHT_IN4 = 23; // Backward
const int MOTOR_RIGHT_ENB = 3; // Speed 3
const int MOTOR_LEFT_IN1 = 26; // Backward
const int MOTOR_LEFT_IN2 = 25; // Forward
const int MOTOR_LEFT_ENA = 2; // Speed

// Ultrasonic sensor
const float TEMPERATURE = 22;//(72 - 32) * 5 / 9;
// Front sensors.
const int TRIG_PIN_FRONT_LOWER = 37;
const int ECHO_PIN_FRONT_LOWER = 36;
AfstandsSensor myFrontLowerSensor(TRIG_PIN_FRONT_LOWER, ECHO_PIN_FRONT_LOWER);
const int TRIG_PIN_RIGHT = 35;
const int ECHO_PIN_RIGHT = 34;
AfstandsSensor myRightSensor(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

// Lidar sensors
const int BACK_PIN = 32;
const int LEFT_PIN_2 = 30;
const int LEFT_PIN = 31;

// Color sensor
const int pinSDA = 40;
const int pinSCL = 41;
const int THRESHOLD = 50;
byte ledColor[] = {0, 0, 0, 0, 0, 0};
byte ledColor2[] = {100, 100, 10, 0, 0, 100};
uint16_t lux = 0;
CustomizedTCS tcs = CustomizedTCS(pinSDA, pinSCL);
uint8_t redValue, greenValue, blueValue;

const int DELAY_TIME = 5000;
const int TURN_INTERVAL = 560;
const int AVOID_INTERVAL = 2000;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

/* Speed constants */
const double HIGH_SPEED = 50;
const double TURN_SPEED = 50;

/* Margins */
const double LOCATION_MARGIN = 0.035;
const double LOCATION_MARGIN_FORWARD = 0.01;
const double GREATER_MARGIN = 0.20;
const double LOCATION_MARGIN_TURN = 0.03;
const double TURN_MARGIN = 0.035;

/* Current location and direction. */
double currentX = 0;
double currentY = 0;
int currentDirec[] = {0, 1}; // This is a unit vector.
double dx, dy, dr, goalX, goalY;

/* Sensor readings */
double backSensor, rightSensor, leftSensor2, leftSensor, adjustment, frontLowerSensor;
int stopStatus = 0;

/* Main functions */
void setup() {
  SPI.begin();
  Serial.begin(9600);
  Wire.begin();
  rest(2000);

  /* Set up pin modes here. */
  setupMotors();
  setupLidars();
  setupSevSeg();
  setupColorSensor();
 
  /* Program starts here. */
  for (int i = 0; i < PLACES_NUMBER - 1; i++) {
    goToNextPlace(i);
  }
  
  goToCustomer(PLACES_NUMBER - 1);
}

void loop() {
  /* We might not need this void loop(), because the project  
   * does not involve an infinite loop, and void loop() cannot be 
   * terminated.
   * We can use while() loop instead in private functions when it is necessary.
   */
}

/* Private functions */

/* Initialize components. */
void setupMotors() {
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);
  pinMode(MOTOR_RIGHT_ENB, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_ENA, OUTPUT);
}

void setupColorSensor() {
  tcs.start();
}

void setupLidars() {
  pinMode(BACK_PIN, OUTPUT);
  pinMode(LEFT_PIN_2, OUTPUT);
  pinMode(LEFT_PIN, OUTPUT);
  digitalWrite(BACK_PIN, LOW);
  digitalWrite(LEFT_PIN_2, LOW);
  digitalWrite(LEFT_PIN, LOW);
  pinMode(BACK_PIN, INPUT);
  distanceSensor.init();
  distanceSensor.setAddress(BACK_PIN);
  distanceSensor.setDistanceMode(VL53L1X::Medium);
  distanceSensor.setMeasurementTimingBudget(50000);
  distanceSensor.startContinuous(50);
  
  pinMode(LEFT_PIN_2, INPUT);
  myLeftSensor2.init();
  myLeftSensor2.setAddress(LEFT_PIN_2);
  myLeftSensor2.setDistanceMode(VL53L1X::Medium);
  myLeftSensor2.setMeasurementTimingBudget(50000);
  myLeftSensor2.startContinuous(50);

  pinMode(LEFT_PIN, INPUT);
  myLeftSensor.init();
  myLeftSensor.setAddress(LEFT_PIN);
  myLeftSensor.setDistanceMode(VL53L1X::Medium);
  myLeftSensor.setMeasurementTimingBudget(50000);
  myLeftSensor.startContinuous(50);
}

void setupSevSeg() {
  byte numDigits = 2;
  byte digitPins[] = {11,12};
  byte segmentPins[] = {4, 5, 6, 7, 8, 9, 10};
  bool resistorsOnSegments = false;
  bool disableDecPoint = true;

  byte hardwareConfig = COMMON_ANODE; 
  sevSeg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments);
  sevSeg.setBrightness(10);
}

/* Method: goToNextPlace() */
/**
 * Move the robot to the next location.
 * @return
 */
void goToNextPlace(int placeNumber) {
  NextPlace myNextPlace = NextPlace(locations[placeNumber], MAP_DATA);
  goalX = myNextPlace.getX();
  goalY = myNextPlace.getY();
  String goalName = myNextPlace.toString();
  
  while(true) {
    
    /* Sensor readings */
    backSensor = readBackSensor(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_VER;
    leftSensor = readLeftSensor(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_HOR;
    leftSensor2 = readLeftSensor2(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_HOR;
    rightSensor = readRightSensor(TEMPERATURE) - WALL_GAP;

    /* Update the current location. */
    updateLocation();

    /* Break when it arrives at the destination. */
    if (dr < LOCATION_MARGIN) {
      stopMoving();
      displayColor(placeNumber);
      displayLocation(goalName);
      break;
    }

    /* Use color sensor to terminate the while loop as well. */
    if (dr < GREATER_MARGIN) {
      getColor();
      if (redValue >= 25 || greenValue >= 18 || blueValue >= 18) {
        stopMoving();
        storeColor(placeNumber);
        setColor(placeNumber);
        displayLocation(goalName);
        break;
      }
    }

    /* Move forward if the current direction is correct. */
    /* Turn 90 degrees if the currect direcion is not correct. */
    /* Avoid turning 180 degrees. */
    if (currentDirec[1] == 1 && dy > LOCATION_MARGIN_FORWARD) {
      if (isFrontClear()) {
        moveForward();
      } else {
        stopMoving();
        turnToAvoid();
        moveTillClear();
      }
    } else if (currentDirec[1] == -1 && -dy > LOCATION_MARGIN_FORWARD) {
      if (isFrontClear()) {
        moveForward();
      } else {
        stopMoving();
        turnToAvoid();
        moveTillClear();
      }
    } else if (currentDirec[0] == 1 && dx > LOCATION_MARGIN_FORWARD) {
      if (isFrontClear()) {
        moveForward();
      } else {
        stopMoving();
        turnToAvoid();
        moveTillClear();
      }
    } else if (currentDirec[0] == -1 && -dx > LOCATION_MARGIN_FORWARD) {
      if (isFrontClear()) {
        moveForward();
      } else {
        stopMoving();
        turnToAvoid();
        moveTillClear();
      }
    } else {
      stopMoving();
      turn90();
    }
    
  }
}

/* Method: goToCustormer() */
void goToCustomer(int placeNumber) {
  NextPlace myNextPlace = NextPlace(locations[placeNumber], MAP_DATA);
  goalX = myNextPlace.getX();
  goalY = myNextPlace.getY();
  String goalName = myNextPlace.toString();

  while(true) {
    /* Sensor readings */
    backSensor = readBackSensor(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_VER;;
    leftSensor = readLeftSensor(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_HOR;
    leftSensor2 = readLeftSensor2(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_HOR;;
    rightSensor = readRightSensor(TEMPERATURE) - WALL_GAP;
    
    /* Update the current location. */
    updateLocation();
    
    /* Break when it arrives at the destination. */
    if (dr < 0.10) {
      stopMoving();
      break;
    }
    
    /* Move forward if the current direction is correct. */
    /* Turn 90 degrees if the currect direcion is not correct. */
    /* Avoid turning 180 degrees. */
    if (currentDirec[1] == 1 && dy > LOCATION_MARGIN_FORWARD) {
      if (isCustomer()) {
        stopMoving();
        break;
      } else {
        moveForward();
      }
    } else if (currentDirec[1] == -1 && -dy > LOCATION_MARGIN_FORWARD) {
      if (isCustomer()) {
        stopMoving();
        break;
      } else {
        moveForward();
      }
    } else if (currentDirec[0] == 1 && dx > LOCATION_MARGIN_FORWARD) {
      if (isCustomer()) {
        stopMoving();
        break;
      } else {
        moveForward();
      }
    } else if (currentDirec[0] == -1 && -dx > LOCATION_MARGIN_FORWARD) {
      if (isCustomer()) {
        stopMoving();
        break;
      } else {
        moveForward();
      }
    } else {
      stopMoving();
      turn90();
    }
  }

  displayLocation(goalName);
  shutDownLED();
}

/* Method: turnToAvoid() */
void turnToAvoid() {
  if (currentDirec[1] == 1) {
    if (isRightWall()) {
      turnLeft();
      currentDirec[0] = -1;
      currentDirec[1] = 0;
    } else {
      turnRight();
      currentDirec[0] = 1;
      currentDirec[1] = 0;
    }
  } else if (currentDirec[1] == -1) {
    if (isRightWall()) {
      turnLeft();
      currentDirec[0] = 1;
      currentDirec[1] = 0;
    } else {
      turnRight();
      currentDirec[0] = -1;
      currentDirec[1] = 0;
    }
  } else if (currentDirec[0] == 1) {
    if (isRightWall()) {
      turnLeft();
      currentDirec[0] = 0;
      currentDirec[1] = 1;
    } else {
      turnRight();
      currentDirec[0] = 0;
      currentDirec[1] = -1;
    }
  } else { // currentDirec[0] == -1
    if (isRightWall()) {
      turnLeft();
      currentDirec[0] = 0;
      currentDirec[1] = -1;
    } else {
      turnRight();
      currentDirec[0] = 0;
      currentDirec[1] = 1;
    }
  }
}

/* Method: turn90() */
void turn90() {
  if (currentDirec[1] == 1 && dx > LOCATION_MARGIN_TURN) {
    turnRight();
    currentDirec[0] = 1;
    currentDirec[1] = 0;
  } else if (currentDirec[1] == 1 && -dx > LOCATION_MARGIN_TURN) {
    turnLeft();
    currentDirec[0] = -1;
    currentDirec[1] = 0;
  } else if (currentDirec[1] == -1 && dx > LOCATION_MARGIN_TURN) {
    turnLeft();
    currentDirec[0] = 1;
    currentDirec[1] = 0;
  } else if (currentDirec[1] == -1 && -dx > LOCATION_MARGIN_TURN) {
    turnRight();
    currentDirec[0] = -1;
    currentDirec[1] = 0;
  } else if (currentDirec[0] == 1 && dy > LOCATION_MARGIN_TURN) {
    turnLeft();
    currentDirec[0] = 0;
    currentDirec[1] = 1;
  } else if (currentDirec[0] == 1 && -dy > LOCATION_MARGIN_TURN) {
    turnRight();
    currentDirec[0] = 0;
    currentDirec[1] = -1;
  } else if (currentDirec[0] == -1 && dy > LOCATION_MARGIN_TURN) {
    turnRight();
    currentDirec[0] = 0;
    currentDirec[1] = 1;
  } else if (currentDirec[0] == -1 && -dy > LOCATION_MARGIN_TURN) {
    turnLeft();
    currentDirec[0] = 0;
    currentDirec[1] = -1;
  }
}

/* Method: moveForward() */
void moveForward() {
  double left = 0;
  double right = 0;
  adjustment = (leftSensor2 - leftSensor) * 200;

  /* Get a smooth start */
  if (stopStatus == 0) {
    stopStatus = 1;
    smoothStart();
  }
  left = abs(HIGH_SPEED - adjustment * 1.5); // Should not send negative PWM (The actual generated PWM goes crazy).
  right = abs(HIGH_SPEED + adjustment * 1.5) + 5;
  if (abs(right - left) > HIGH_SPEED / 3) {
    left = HIGH_SPEED;
    right = HIGH_SPEED + 5;
  }
  keepMoving(left, right);
}

/* Method: smoothStart() */
void smoothStart() {
  double currentSpeed = 10;
  while(true) {
    updateLocation();
    keepMoving(currentSpeed, currentSpeed + 5);
    currentSpeed += 1.5;
    rest(5);
    
    if (currentSpeed >= HIGH_SPEED / 2) {
      break;
    }

    if (dr < GREATER_MARGIN) {
      getColor();
      if (redValue >= 19 || greenValue >= 18 || blueValue >= 18) {
        stopMoving();
        break;
      }
    }
  }
}

/* Method: moveTillClear() */
void moveTillClear() {
  previousMillis = millis();
  while(true) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= AVOID_INTERVAL) {
      break;
    }
    moveForward();
    if (!isFrontClear()) {
      break;
    }
  }
  stopMoving();
}

/* Method: turnRight() */
/* Using a timer */
void turnRight() {
  previousMillis = millis(); // Start timing.
  double back = 0;
  double front = 0;
  while(true) {
    leftSensor = readLeftSensor(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_HOR;
    leftSensor2 = readLeftSensor2(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_HOR;
    back = readBackSensor(10);
    front = readFrontLowerSensor(10);
    currentMillis = millis();
    if (currentMillis - previousMillis >= TURN_INTERVAL) {
      if (abs(leftSensor2 - leftSensor) <= TURN_MARGIN || abs(MAP_WIDTH - back - front - 0.21) <= 0.05) {
        break;
      }
    }
    keepTurningRight();
  }
  stopMoving();
}

/* Method: keepTurningRight() */
void keepTurningRight() {
  analogWrite(MOTOR_RIGHT_ENB, TURN_SPEED + 5);
  analogWrite(MOTOR_LEFT_ENA, TURN_SPEED + 10);
  digitalWrite(MOTOR_RIGHT_IN3, LOW); // Forward
  digitalWrite(MOTOR_RIGHT_IN4, HIGH); // Backward
  digitalWrite(MOTOR_LEFT_IN1, LOW); // Backward
  digitalWrite(MOTOR_LEFT_IN2, HIGH); // Forward
}

/* Method: turnLeft() */
/* Using a timer. */
void turnLeft() {
  previousMillis = millis(); // Start timing.
  double front = 0;
  double back = 0;
  while(true) {
    leftSensor = readLeftSensor(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_HOR;
    leftSensor2 = readLeftSensor2(TEMPERATURE) - WALL_GAP + COLORSENSOR_POS_HOR;
    back = readBackSensor(10);
    front = readFrontLowerSensor(10);
    currentMillis = millis();
    if (currentMillis - previousMillis >= TURN_INTERVAL) {
      if (abs(leftSensor2 - leftSensor) <= TURN_MARGIN || abs(MAP_WIDTH - back - front - 0.21) <= 0.05) {
        break;
      }
    }
    keepTurningLeft();
  }
  stopMoving();
}

/* Method: keepTurningLeft() */
void keepTurningLeft() {
  analogWrite(MOTOR_RIGHT_ENB, TURN_SPEED + 5);
  analogWrite(MOTOR_LEFT_ENA, TURN_SPEED + 10);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
}

/* Method: stopMoving() */
void stopMoving() {
  stopStatus = 0;
  analogWrite(MOTOR_RIGHT_ENB, 0);
  analogWrite(MOTOR_LEFT_ENA, 0);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  displayCurrentLocation(getCurrentBlock());
}

/* Method: keepMoving() */
void keepMoving(double speedLeft, double speedRight) {
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
  analogWrite(MOTOR_RIGHT_ENB, speedRight);
  analogWrite(MOTOR_LEFT_ENA, speedLeft);
}

/* Method: displayLocation() */
void displayLocation(String str) {
  int strLen = str.length() + 1;
  char charArray[strLen];
  str.toCharArray(charArray, strLen);
  previousMillis = millis();
  while(true) {
    currentMillis = millis();
    sevSeg.refreshDisplay();
    sevSeg.setChars(charArray);
    if (currentMillis - previousMillis >= DELAY_TIME) {
      break;
    }
  }
}

/* Method: displayCurrentLocation() */
void displayCurrentLocation(String str) {
  int strLen = str.length() + 1;
  char charArray[strLen];
  str.toCharArray(charArray, strLen);
  previousMillis = millis();
  while(true) {
    currentMillis = millis();
    sevSeg.refreshDisplay();
    sevSeg.setChars(charArray);
    if (currentMillis - previousMillis >= 500) {
      break;
    }
  }
}

/* Method: getCurrentBlock() */
String getCurrentBlock() {
  char x = ((int) (currentX / (MAP_WIDTH / BLOCK_NUMBERS))) + '1';
  char y = ((int) (currentY / (MAP_WIDTH / BLOCK_NUMBERS))) + 'A';
  String str = "";
  str += y;
  str += x;
  return str;
}

/* Method: getOrientation */
String getOrientation() {
  String str = "";
  str += currentDirec[0];
  str += currentDirec[1];
  return str;
}

/* Method: displayColor() */
void displayColor(int i) {
  getColor();
  storeColor(i);
  setColor(i);
}

/* Method: isCustomer */
boolean isCustomer() {
  if (readFrontLowerSensor(TEMPERATURE) <= 2.75 / 39.37) { // 1.0 inches
    return true;
  } else {
    return false;
  }
}

/* Method: isFrontClear() */
boolean isFrontClear() {
  frontLowerSensor = readFrontLowerSensor(TEMPERATURE);
  if (frontLowerSensor <= 8 / 39.37 && frontLowerSensor > 0) { // 2.5 inches
    return false;
  } else {
    return true;
  }
}

/* Method: isRightWall() */
boolean isRightWall() {
  if (rightSensor <= 1.0 / 3.2808 && rightSensor > 0) {
    return true;
  } else {
    return false;
  }
}

/* Method: updateLocation() */
/* This is the color sensor's location. */
void updateLocation() {
  if (currentDirec[1] == 1) {
    currentY = backSensor;
    currentX = leftSensor;
  } else if (currentDirec[1] == -1) {
    currentY = MAP_WIDTH - backSensor;
    currentX = MAP_WIDTH - leftSensor;
  } else if (currentDirec[0] == 1) {
    currentX = backSensor;
    currentY = MAP_WIDTH - leftSensor;
  } else { // currentDirec[0] == -1
    currentX = MAP_WIDTH - backSensor;
    currentY = leftSensor;
  }

  dx = goalX - currentX;
  dy = goalY - currentY;
  dr = sqrt(dx * dx + dy * dy);
}

/* Method: readBackSensor() */
double readBackSensor(float t) {
  return distanceSensor.read() / 1000.0;
}

/* Method: readLeftSensor2 */
double readLeftSensor2(float t) {
  /* Adjust this sensor reading to leftSensor reading. */
  /* y = 0.9838x-0.0045 */
  /* y = -0.1457x6 + 1.0332x5 - 2.7475x4 + 3.4276x3 - 2.0888x2 + 1.579x - 0.0658 */
  double x = myLeftSensor2.read() / 1000.0;
  x = -0.1457 * x * x * x * x * x * x + 1.0332 * x * x * x * x * x - 2.7475 * x * x * x * x + 3.4276 * x * x * x - 2.0888 * x * x + 1.579 * x - 0.0658;
  if (x >= 1.8) {
    x += 0.05;
  } else if (x >= 1.3) {
    x += 0.04;
  } else if (x >= 1.2) {
    x += 0.03;
  } else if (x >= 0.7) {
    x += 0.02;
  } else if (x >= 0.55) {
    x += 0.016;
  }
  return x;
}

/* Method: readFrontLowerSensor */
double readFrontLowerSensor(float t) {
  return myFrontLowerSensor.afstandCM(t) / 100.0;
}

/* Method: readRightSensor */
double readRightSensor(float t) {
  return myRightSensor.afstandCM(t) / 100.0;
}

/* Method: readLeftSensor */
double readLeftSensor(float t) {
  return myLeftSensor.read() / 1000.0;
}

/* Method: rest */
void rest(unsigned long restTime) {
  unsigned long pre = 0;
  unsigned long now = 0;
  pre = millis();
  while(true) {
    now = millis();
    if (now - pre >= restTime) {
      break;
    }
  }
}

/* Essential functions for LED strips */
void getColor() {
  lux = tcs.getRGB(&redValue, &greenValue, &blueValue);
  if (redValue + greenValue + blueValue < 50) {
    redValue = 0;
    greenValue = 0;
    blueValue = 0;
  }
}

void storeColor(int i) {
  if (i == 0) {
    ledColor[0] = redValue;
    ledColor[1] = greenValue;
    ledColor[2] = blueValue;
  }
  if (i == 1) {
    ledColor[3] = redValue;
    ledColor[4] = greenValue;
    ledColor[5] = blueValue;
  }
}

void setColor(int i) {
  if (i == 0) {
    sendStartFrame();
    sendColorFrame(16, ledColor[0], ledColor[1], ledColor[2]);
    sendColorFrame(0, 0, 0, 0);
    sendColorFrame(0, 0, 0, 0);
    sendColorFrame(0, 0, 0, 0);
    sendColorFrame(0, 0, 0, 0);
    sendEndFrame();
  }
  if (i == 1) {
    sendStartFrame();
    sendColorFrame(16, ledColor[3], ledColor[4], ledColor[5]);
    sendColorFrame(16, ledColor[0], ledColor[1], ledColor[2]);
    sendColorFrame(0, 0, 0, 0);
    sendColorFrame(0, 0, 0, 0);
    sendColorFrame(0, 0, 0, 0);
    sendEndFrame();
  }
}

void sendStartFrame() {
  SPI.transfer(B00000000);
  SPI.transfer(B00000000);
  SPI.transfer(B00000000);
  SPI.transfer(B00000000);
}

void sendEndFrame() {
 // Your code here  
  SPI.transfer(B11111111);
  SPI.transfer(B11111111);
  SPI.transfer(B11111111);
  SPI.transfer(B11111111);
}

void sendColorFrame(byte brightness, byte R, byte G, byte B) {
 // Your code here
  int key = B11100000;
  brightness = brightness | key;
  SPI.transfer(brightness);
  SPI.transfer(B);
  SPI.transfer(G);
  SPI.transfer(R);
}

void shutDownLED() {
  sendStartFrame();
  sendColorFrame(0, 0, 0, 0);
  sendColorFrame(0, 0, 0, 0);
  sendColorFrame(0, 0, 0, 0);
  sendColorFrame(0, 0, 0, 0);
  sendColorFrame(0, 0, 0, 0);
  sendEndFrame();
}
