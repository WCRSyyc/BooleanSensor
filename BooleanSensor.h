/*
  BooleanSensor.h - Analog input as boolean library for Arduino
  Copyright (c) 2015 H. Phil Duby.  All rights reserved.

  This software is licensed under the MIT License.  See the LICENSE document
  included in the same folder as this file.
*/

/*
#define SENSE_PIN A0
BooleanSensor testSensor = BooleanSensor(SENSE_PIN);
void setup() {}
void loop() {
  boolean pinState = testSensor.read();
}

The read method tracks the minimum and maximum values of all previous reads, and
uses that to set the threshold midpoint and deadband.
*/

#ifndef BooleanSensor_h
#define BooleanSensor_h

#include <Arduino.h>

#define BooleanSensor_VERSION    1      // software version of this library
// #define DEBUG 1

typedef struct {
  uint16_t lowerBound;
  uint16_t upperBound;
} range_t;

class BooleanSensor
{
public:
  // Constructor
  BooleanSensor(uint8_t pin);

  boolean read();
private:
  uint8_t sensorPin; // Arduino analog pin number
  boolean boolState;
  range_t readRange;
  range_t threshold;
  void rangeToThreshold();
  void analogToBoolean(uint16_t value);
};

#endif /* BooleanSensor.h */
