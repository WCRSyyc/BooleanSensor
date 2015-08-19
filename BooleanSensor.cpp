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
// have a single input but with multiple (manually configured) thresholds

const uint16_t MAX_ADC = 1023;// ADC == analog to digital conversion
const uint16_t MIN_ADC = 0;
const float DEAD_BAND_FACTOR = 0.25;// +/- 25% of the size of the complete range
const uint16_t FIXED_DEAD_BAND = 20;// +/- 20 should be plenty to prevent jitter/bounce
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
  Serial.print("  deadSideBandSize: ");
  Serial.print(deadSideBandSize);
  Serial.println(",");
  Serial.print("  deadSideBandFraction: ");
  Serial.print(deadSideBandFraction);
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
  readMode == 0 ? configRead() : fastRead();
  return boolState;
}// end boolean BooleanSensor::read()
// boolean BooleanSensor::fastRead(BooleanSensor sensorInstance) {
//   return sensorInstance.analogToBoolean(analogRead(sensorInstance.sensorPin));
// }
inline void BooleanSensor::fastRead() {
  analogToBoolean(analogRead(sensorPin));
}
inline void BooleanSensor::configRead()
{
  boolean newRange = false;
#ifdef DEBUG
  range_t prevRange = readRange;
  range_t prevThreshold = threshold;
#endif /* DEBUG */
  uint16_t raw;
  raw = analogRead(sensorPin);
  // readRange.lowerBound = min(readRange.lowerBound, raw);
  // readRange.upperBound = max(readRange.upperBound, raw);
  // rangeToThreshold();
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
    rangeToThreshold();
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
  analogToBoolean(raw);
}// end inline void BooleanSensor::configRead()

inline void BooleanSensor::analogToBoolean(uint16_t value)
{
  if (threshold.upperBound - threshold.lowerBound < MIN_DEADBAND) {
    // Do not toggle state until have seen enough input range to create a
    // minimally valid dead band
    return;
  }

  if (value < threshold.lowerBound) {
    // value less than the (lower boundary of the) threshold
    boolState = false;
  }
  if (value > threshold.upperBound) {
    // value greater than the (upper boundary of the) threshold
    boolState = true;
  }
}// end of inline void BooleanSensor::analogToBoolean(uint16_t value)

// Determine new dead band threshold limits based on the provided range of input values
inline void BooleanSensor::rangeToThreshold()
{
  unsigned int median, halfBand;
  // Get the midpoint between the minimum and maximum values to use as
  // the base threshold between LOW and HIGH ADC values.
  median = (readRange.upperBound + readRange.lowerBound) / 2;
  // Pick a dead-band size where the boolean state will not change.
  halfBand = sideBandSize();

  // Set the threshold with the dead band range boundary limits
  threshold.lowerBound = max(MIN_ADC, median - halfBand);
  threshold.upperBound = min(MAX_ADC, median + halfBand);
#ifdef DEBUG
  Serial.print("rangeToThreshold: range ");
  Serial.print(readRange.lowerBound);
  Serial.print("..");
  Serial.print(readRange.upperBound);
  Serial.print(" ==> threshold ");
  Serial.print(threshold.lowerBound);
  Serial.print("..");
  Serial.print(threshold.upperBound);
  Serial.print(" at time ");
  Serial.print(micros());
  Serial.print(" from halfBand ");
  Serial.print(halfBand);
  Serial.print(" with midpoint ");
  Serial.println(median);
#endif /* DEBUG */
}// end void BooleanSensor::rangeToThreshold()

// Calculate (half of the) size of the threshold dead band, based on the supplied
// input range, and instance configuration
inline uint16_t BooleanSensor::sideBandSize()
{
  unsigned int extent;
  extent = readRange.upperBound - readRange.lowerBound;// Size of the range

  switch (deadBandMode) {
    case DEAD_BAND_FIXED:
      return deadSideBandSize;
      break;
    case DEAD_BAND_FRACTION:
      return extent * deadSideBandFraction;
      break;
    // default: //DEAD_BAND_MINIMUM
  }
  return min(extent * deadSideBandFraction, deadSideBandSize);
}// end inline uint16_t BooleanSensor::sideBandSize(range_t range)
