/*
  BooleanSensor.cpp - Analog input as boolean library for Arduino
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

#include <Arduino.h>

#include "BooleanSensor.h"

const uint16_t MAX_ADC = 1023;// ADC == analog to digital conversion
const uint16_t MIN_ADC = 0;
const uint16_t SIDE_DEAD_BAND = 20;// +/- 20 should be plenty to prevent jitter/bounce
const uint16_t MIN_DEADBAND = 5;// prevent jitter while doing initial calibration

/************ static functions common to all instances ***********************/

/****************** end of static functions ******************************/

// Constructor
BooleanSensor::BooleanSensor(uint8_t pin)
{
  sensorPin = pin;
  boolState = true;
  readRange.lowerBound = MAX_ADC;
  readRange.upperBound = MIN_ADC;
  threshold.lowerBound = MAX_ADC;
  threshold.upperBound = MIN_ADC;
}// end BooleanSensor::BooleanSensor(uint8_t pin)

boolean BooleanSensor::read()
{
  boolean newRange = false;
  uint16_t raw;
  raw = analogRead(sensorPin);
  if (raw < readRange.lowerBound) {
    readRange.lowerBound = raw;
    newRange = true;
  }
  if (raw > readRange.upperBound) {
    readRange.upperBound = raw;
    newRange = true;
  }
  if (newRange) {
    rangeToThreshold();
  }
  analogToBoolean(raw);
  return boolState;
}// end boolean BooleanSensor::read()

inline void BooleanSensor::analogToBoolean(uint16_t value)
{
  if (threshold.upperBound - threshold.lowerBound < MIN_DEADBAND) {
    // Do not toggle state until have seen enough input range to create a
    // minimally valid dead band
    return;
  }

  if (value < threshold.lowerBound) {
    // new value less than (low end of) deadband
    boolState = false;
  }
  if (value > threshold.upperBound) {
    // new value greater than (high end of) deadband
    boolState = true;
  }
}// end inline boolean BooleanSensor::analogToBoolean(uint16_t value)

// Determine new dead band threshold limits based on the provided range of input values
inline void BooleanSensor::rangeToThreshold()
{
  unsigned int median;
  // Get the midpoint between the minimum and maximum values to use as
  // the base threshold between LOW and HIGH ADC values.
  median = (readRange.upperBound + readRange.lowerBound) / 2;

  // set the deadband where the boolean state will not change
  threshold.lowerBound = max(MIN_ADC, median - SIDE_DEAD_BAND);
  threshold.upperBound = min(MAX_ADC, median + SIDE_DEAD_BAND);
}// end inline BooleanSensor::rangeToThreshold()
