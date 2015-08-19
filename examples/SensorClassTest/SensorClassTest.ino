/* SensorClassTest

  Boolean state for an analog sensor

  Use (under development) library to handle an analog sensor where the
  result is to be turned into a boolean HIGH / LOW, true / false result, with
  suppression of spurious transitions due to analog sensor jitter near the cross
  over voltage.
*/
#include <BooleanSensor.h>

unsigned const int SERIAL_SPEED = 9600;
//300, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200

unsigned const int SENSE_PIN = A0;
// BooleanSensor testSensor = BooleanSensor();
BooleanSensor testSensor = BooleanSensor(SENSE_PIN);

boolean prevState = HIGH;
unsigned long readCount = 0;
unsigned long baseTime;
#ifdef DEBUG
boolean firstLoop = true;
#endif /* DEBUG */

void setup() {
  Serial.begin(SERIAL_SPEED);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  baseTime = micros();
}

void loop() {
  boolean test;
#ifdef DEBUG
  if (firstLoop) {
    firstLoop = false;
    testSensor.dump();
  }
#endif /* DEBUG */
  test = testSensor.read();
  if (test != prevState) {
    prevState = test;
    Serial.print("State = ");
    Serial.print(test);
    Serial.print(" at ");
    Serial.print(micros());
    Serial.println(" microseconds");

  }
  readCount += 1;
  if (micros() - baseTime > 1000000) {
    Serial.print(readCount);
    Serial.println(" sensor reads in the past 1 second");
    readCount = 0;
    baseTime = micros();
  }
}
