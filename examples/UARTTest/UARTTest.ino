#include <Arduino.h>

HardwareSerial& testserial = Serial1;

void setup() {
  testserial.begin(9600);
}

void loop() {
  testserial.println("Test");
  delay(1000);
}