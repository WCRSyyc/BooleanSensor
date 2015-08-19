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

enum {
  DEAD_BAND_MINIMUM,
  DEAD_BAND_FIXED,
  DEAD_BAND_FRACTION
};

class BooleanSensor
{
public:
  // Constructors
  BooleanSensor(uint8_t pin, uint16_t deadband);
  BooleanSensor(uint8_t pin);

  typedef boolean (BooleanSensor::*BooleanSensorMFP)();
  // read boolean (state) value from analog sensor
  // boolean (BooleanSensor::*read)();
  // BooleanSensorMFP read;
  boolean read();


#ifdef DEBUG
  void dump();
#endif /* DEBUG */
private:
  uint8_t sensorPin; // Arduino analog pin number
  uint8_t deadBandMode; //
  boolean boolState;
  uint16_t deadSideBandSize;
  float deadSideBandFraction;
  range_t readRange;
  range_t threshold;
  void init(uint8_t pin, uint16_t deadband);
  range_t rangeToThreshold(range_t range);
  boolean analogToBoolean(uint16_t value);
  uint16_t sideBandSize(range_t range);
  // static boolean fastRead(BooleanSensor sensorInstance);
  // static boolean configRead(BooleanSensor sensorInstance);
  boolean fastRead();
  boolean configRead();
  // boolean (*actualRead)();
  // boolean (BooleanSensor::*actualRead)();
  uint8_t readMode;
};

#endif /* BooleanSensor.h */
