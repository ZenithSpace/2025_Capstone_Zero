#include "led_controller.h"

void LedController::setup() {
    pinMode(RED, OUTPUT);
    pinMode(YELLOW, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
}

void LedController::driveManual() {
    digitalWrite(RED, LOW);
    digitalWrite(YELLOW, LOW);
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, HIGH);
}

void LedController::estop() {
    digitalWrite(RED, LOW);
    digitalWrite(GREEN, LOW);
    digitalWrite(BLUE, LOW);

    digitalWrite(YELLOW, HIGH);
    delay(500);
    digitalWrite(YELLOW, LOW);
    delay(500);
}
