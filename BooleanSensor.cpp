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

// TODO: QUERY: how to determine how many ADC pins exist for the
// current board? Use as array size for the used pins to prevent
// multiple sensors on the same pin
// Should mutiple instance on the same pin be prevented?  Could potentially
// have a single input but with multiple thresholds

const uint16_t MAX_ADC = 1023;// ADC == analog to digital conversion
const uint16_t MIN_ADC = 0;
const float DEAD_BAND_FACTOR = 0.25;// +/- 25% of the size of the complete range
const uint16_t FIXED_DEAD_BAND = 40;// +/- 40 should be plenty to prevent jitter/bounce
const uint16_t MIN_DEADBAND = 5;// prevent jitter while doing initial calibration

/************ static functions common to all instances ***********************/

/****************** end of static functions ******************************/

// Constructors
BooleanSensor::BooleanSensor(uint8_t pin, uint16_t deadband)
{
  init(pin, deadband);
}// end BooleanSensor::BooleanSensor(uint8_t pin, uint16_t deadband)
BooleanSensor::BooleanSensor(uint8_t pin)
{
  init(pin, FIXED_DEAD_BAND);
}// end BooleanSensor::BooleanSensor(uint8_t pin)

void BooleanSensor::init(uint8_t pin, uint16_t deadband)
{
  sensorPin = pin;
  deadSideBandSize = deadband;
  deadSideBandFraction = DEAD_BAND_FACTOR;
  boolState = true;
  readRange.lowerBound = MAX_ADC;
  readRange.upperBound = MIN_ADC;
  threshold.lowerBound = MAX_ADC;
  threshold.upperBound = MIN_ADC;
  deadBandMode = DEAD_BAND_MINIMUM;
  // actualRead = BooleanSensor::configRead;
  readMode = 0;
}// end void BooleanSensor::init(uint8_t pin, uint16_t deadband)

#ifdef DEBUG
void BooleanSensor::dump()
{
  Serial.println("BooleanSensor::dump {");
  Serial.print("  sensorPin: ");
  Serial.print(sensorPin);
  Serial.println(",");
  Serial.print("  deadBandMode: ");
  Serial.print(deadBandMode);
  Serial.println(",");
  Serial.print("  readMode: ");
  Serial.print(readMode);
  Serial.println(",");
  Serial.print("  readRange: ");
  Serial.print(readRange.lowerBound);
  Serial.print("..");
  Serial.print(readRange.upperBound);
  Serial.println(",");
  Serial.print("  threshold: ");
  Serial.print(threshold.lowerBound);
  Serial.print("..");
  Serial.print(threshold.upperBound);
  Serial.println(",");
  Serial.print("  boolState: ");
  Serial.println(boolState);
  Serial.println("}");
}// end void BooleanSensor::dump() {
#endif /* DEBUG */

boolean BooleanSensor::read()
{
  // return actualRead(this);
  // return *actualRead(this);
  // return ((BooleanSensor*)this)->actualRead(this);
  // return ((BooleanSensor*)this)->BooleanSensor::actualRead(this);
  // SIGH
  return readMode == 0 ? configRead() : fastRead();
}// end boolean BooleanSensor::read()
// boolean BooleanSensor::fastRead(BooleanSensor sensorInstance) {
//   return sensorInstance.analogToBoolean(analogRead(sensorInstance.sensorPin));
// }
boolean BooleanSensor::fastRead() {
  return analogToBoolean(analogRead(sensorPin));
}
// boolean BooleanSensor::configRead(BooleanSensor sensorInstance) {
//   uint16_t raw;
//   raw = analogRead(sensorInstance.sensorPin);
//   if (raw < sensorInstance.readRange.lowerBound) {
//     sensorInstance.readRange.lowerBound = raw;
//     sensorInstance.rangeToThreshold(sensorInstance.readRange);
//   }
//   if (raw > sensorInstance.readRange.upperBound) {
//     sensorInstance.readRange.upperBound = raw;
//     sensorInstance.rangeToThreshold(sensorInstance.readRange);
//   }
//   return sensorInstance.analogToBoolean(raw);
// }
boolean BooleanSensor::configRead() {
  boolean newRange = false;
#ifdef DEBUG
  range_t prevRange = readRange;
  range_t prevThreshold = threshold;
#endif /* DEBUG */
  uint16_t raw;
  raw = analogRead(sensorPin);
  // readRange.lowerBound = min(readRange.lowerBound, raw);
  // readRange.upperBound = max(readRange.upperBound, raw);
  // threshold = rangeToThreshold(readRange);
  if (raw < readRange.lowerBound) {
    readRange.lowerBound = raw;
    newRange = true;
  }
  if (raw > readRange.upperBound) {
    readRange.upperBound = raw;
    newRange = true;
  }
  if (newRange) {
#ifdef DEBUG
    Serial.print("configRead: range ");
    Serial.print(prevRange.lowerBound);
    Serial.print("..");
    Serial.print(prevRange.upperBound);
    Serial.print(" ==> ");
    Serial.print(readRange.lowerBound);
    Serial.print("..");
    Serial.print(readRange.upperBound);
    Serial.print(" for raw value ");
    Serial.print(raw);
    Serial.print(" at time ");
    Serial.println(micros());
#endif /* DEBUG */
    threshold = rangeToThreshold(readRange);
#ifdef DEBUG
    Serial.print("configRead: threshold ");
    Serial.print(prevThreshold.lowerBound);
    Serial.print("..");
    Serial.print(prevThreshold.upperBound);
    Serial.print(" ==> ");
    Serial.print(threshold.lowerBound);
    Serial.print("..");
    Serial.println(threshold.upperBound);
#endif /* DEBUG */
  }
  return analogToBoolean(raw);
}

boolean BooleanSensor::analogToBoolean(uint16_t value) {
  boolean curState = boolState;
  if (threshold.upperBound - threshold.lowerBound < MIN_DEADBAND) {
    // Do not toggle state until have seen enough input range to create a
    // minimally valid dead band
    return boolState;
  }

  if (boolState) {
    if (value < threshold.lowerBound) {
      // Previous state was true/HIGH, and now less than the lower threshold boundary
      curState = false;
    }
  } else {
    if (value > threshold.upperBound) {
      // Previous state was false/LOW, and now greater than the upper threshold boundary
      curState = true;
    }
  }
#ifdef DEBUG
  if (curState != boolState) {
    Serial.print(curState ? "^" : "v");
  }
#endif /* DEBUG */
  boolState = curState;
  return boolState;
}

// Determine new dead band threshold limits based on the provided range of input values
range_t BooleanSensor::rangeToThreshold(range_t inputRange) {
  range_t thresholdRange;
  unsigned int median, halfBand;
  // Get the midpoint between the minimum and maximum values to use as
  // the base threshold between LOW and HIGH ADC values.
  median = (inputRange.upperBound + inputRange.lowerBound) / 2;
  // Pick a dead-band size where the boolean state will not change.
  halfBand = sideBandSize(inputRange);

  // Set the threshold with the dead band range boundary limits
  thresholdRange.lowerBound = max(MIN_ADC, median - halfBand);
  thresholdRange.upperBound = min(MAX_ADC, median + halfBand);
#ifdef DEBUG
  Serial.print("rangeToThreshold: range ");
  Serial.print(readRange.lowerBound);
  Serial.print("..");
  Serial.print(readRange.upperBound);
  Serial.print(" ==> threshold ");
  Serial.print(thresholdRange.lowerBound);
  Serial.print("..");
  Serial.print(thresholdRange.upperBound);
  Serial.print(" at time ");
  Serial.print(micros());
  Serial.print(" from halfBand ");
  Serial.print(halfBand);
  Serial.print(" with midpoint ");
  Serial.println(median);
#endif /* DEBUG */
  return thresholdRange;
}

// Calculate (half of the) size of the threshold dead band, based on the supplied
// input range, and instance configuration
uint16_t BooleanSensor::sideBandSize(range_t range) {
  unsigned int extent;
  extent = range.upperBound - range.lowerBound;// Size of the range

  switch (deadBandMode) {
    case DEAD_BAND_FIXED:
      return deadSideBandSize;
      break;
    case DEAD_BAND_FRACTION:
      return extent * deadSideBandFraction;
      break;
    default: //DEAD_BAND_MINIMUM
      return min(extent * deadSideBandFraction, deadSideBandSize);
  }
}
