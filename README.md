# BooleanSensor

Arduino library to use Analog inputs as boolean values with (automatic) threshold
calibration and jitter (bounce) filtering.  Useful for things like line sensing.

Instances created from the BooleanSensor class in this library track the history of
analog reads for the configured pin number, and use the range of values seen to
calculate a threshold midpoint, then adds a [deadband](https://en.wikipedia.org/wiki/Deadband)
around it to reduce spurious (boolean) state transitions if/when the analog readings
jitter around the mid point.  The state will only change when the latest reading is
outside of the threshold range.  That is, when the current state is LOW (false), the
state will only change to HIGH (true) when a reading is seen that is greater than
the midpoint plus half the size of the deadband.  When the current state is HIGH,
the state will only change to LOW when a reading is seen that is less than the
midpoint minus half the size of the deadband.

Usage:
```
BooleanSensor testSensor = BooleanSensor(SENSE_PIN);
void setup() {}
void loop() {
  boolean pinState = testSensor.read();
}
```
