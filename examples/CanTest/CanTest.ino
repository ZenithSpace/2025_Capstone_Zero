#include <Arduino_CAN.h>
#include "zltech_controller.h"
#include "ZLAC8015D_OD.h"

// === Motor Driver ===
#define NODE_ID 1
zltech_controller zltech(NODE_ID);
CanMsg zltech_msg;

bool is_motor_connected = false;

void setup() {
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  zltech.init();
  zltech.setup();
  is_motor_connected = zltech.isConnected();
  Serial.println(is_motor_connected);
}

void loop() {
  // put your main code here, to run repeatedly:

}
