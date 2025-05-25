#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>

#define RED PI_3
#define YELLOW PI_2
#define GREEN PI_7
#define BLUE PI_5

class LedController {
public:
    void setup();
    void driveManual();
    void estop();

private:

};

#endif
