#include <step_controller.h>

#define STEP1_ID 0x01

StepController stepmotor_1(STEP1_ID);

void setup() {
  Serial.begin(115200);
  stepmotor_1.init();

  

  // stepmotor_1.sendAngle(0.1745329252);

}

void loop() {
  stepmotor_1.readEncoderValue();
  
}
