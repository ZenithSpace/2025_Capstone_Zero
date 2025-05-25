#include "rf_controller_dd_robot.h"
#include "pc_comm.h"
#include "state.h"
#include "gear.h"
#include "drive_mode.h"
#include "led_controller.h"

#include <Arduino_CAN.h>
#include "zltech_controller.h"
#include "ZLAC8015D_OD.h"

#include "mbed.h"

#include <cmath>  //atan2
#include "step_controller.h"

// === Motor Driver ===
#define NODE_ID 1
zltech_controller zltech(NODE_ID);
CanMsg zltech_msg;

// === Step Motor ===
#define STEP1_ID 0x01
#define STEP2_ID 0x02
#define STEP3_ID 0x03
#define STEP4_ID 0x04
// #define GROUP_STEP12_ID 0x50
// #define GROUP_STEP34_ID 0x51
StepController stepmotor_1(STEP1_ID);   // Front Left
StepController stepmotor_2(STEP2_ID);   // Back Left
StepController stepmotor_3(STEP3_ID);   // Front Right
StepController stepmotor_4(STEP4_ID);   // Back Right

// === Robot Hardware ===
#define WHEEL_R  0.09394
#define WHEELBASE  0.6638
#define THREAD 0.517

// === Portenta Hardware ===
#define LED_PIN LED_BUILTIN
#define SERIAL_RF Serial3
#define SERIAL_PC Serial
#define SERIAL_DEBUG Serial1
#define PIN_ESTOP 6
#define voltagePin A0

// === RF Controller Channel ===
#define CH_VELOCITY 1
#define CH_OMEGA 3
#define CH_DISCONNECT 4  //6
#define CH_DRIVE_MODE 5  //7
#define CH_GEAR 6  //8
#define CH_ESTOP 7

// === Variables ===
State state = INIT;
Gear gear = GEAR_NEUTRAL;
DriveMode drive_mode = MODE_MANUAL;
uint8_t estop = 0;
int velocity = 0;
int omega = 0;
uint8_t heartbeatState = 255;
int32_t left_actual_rpm = 0;
int32_t right_actual_rpm = 0;
byte brake = 0;
uint8_t battery = 140;

RFControllerDDRobot rf_controller(SERIAL_RF);
PcComm upper_comm;
LedController led_controller;

bool is_estop = false;
bool is_estop_btn_on = false;
bool is_estop_rf_on = false;
bool is_rf_disconnected = false;
bool is_upper_connected = false;
bool is_motor_connected = false;
bool is_battery_low = false;

// === Threads & Mutex ===
rtos::Thread threadRF; // RF Controller Thread
rtos::Thread threadRobotControl; // Motor Control Thread
rtos::Thread threadUpperComm; // PC <-> Portenta Communication Thread
rtos::Thread threadBattery; // Battery check
rtos::Mutex dataMutex;

#define FLAG_RF (1UL << 0)
#define FLAG_ROBOT_CONTROL (1UL << 1)
#define FLAG_UPPER_COMM (1UL << 2)
#define FLAG_BATTERY (1UL << 3)

mbed::Ticker tick1ms;   /* 10 ms*/
uint32_t tick_cnt = 0;
const uint8_t tick_robot_control = 10; // 10 ms
const uint8_t tick_rf = 20; // 20 ms
const uint8_t tick_upper = 2; // 2 ms
const uint8_t tick_battery = 1000; // 1000 ms
rtos::EventFlags ef;

void setup() {  
  // while(!SERIAL_PC){
  //   ;
  // }


  SERIAL_DEBUG.begin(9600);
  delay(50);

  upper_comm.begin(SERIAL_PC);
  delay(50);

  state = MOTOR_IDLE;

  stepmotor_1.init();
  stepmotor_2.init();
  stepmotor_3.init();
  stepmotor_4.init();

  // stepmotor_1.setGroupID(GROUP_STEP12_ID);
  // stepmotor_2.setGroupID(GROUP_STEP12_ID);
  // stepmotor_3.setGroupID(GROUP_STEP34_ID);
  // stepmotor_4.setGroupID(GROUP_STEP34_ID);
  
  zltech.init();
  zltech.setup();
  is_motor_connected = zltech.isConnected();
  delay(50);
  
  state = RF_IDLE;
  rf_controller.setMaxOmega(30.0); // [deg]
  rf_controller.setChannel(CH_ESTOP, CH_DISCONNECT, CH_GEAR, CH_DRIVE_MODE, CH_VELOCITY, CH_OMEGA); // estop, disconnect, gear, drive_mode, throttle, steer,  channels
  rf_controller.begin();

  state = DRIVE_READY;

  // pinMode(PIN_ESTOP, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), detectEstopSW, CHANGE);

  // Thread
  tick1ms.attach(&setFlag, 0.001); // 1 ms

  threadRF.start(taskRF);
  threadRobotControl.start(taskRobotControl);
  threadUpperComm.start(taskUpperComm);
  threadBattery.start(taskBattery);
}

void taskRF(){
  while(true){
    ef.wait_any(FLAG_RF);

    is_rf_disconnected = !rf_controller.isConnected();

    if (! is_rf_disconnected) {
      is_estop_rf_on = rf_controller.getEstop();
      drive_mode = rf_controller.getDriveMode();

      rf_controller.getVelocity(&velocity);

      omega = rf_controller.getOmegaAngle() ;
      gear = rf_controller.getGear();
    }

    // SERIAL_PC.print("[");
    // SERIAL_PC.print(!is_rf_disconnected ? "RECEIVED" : "DISCONNECTED");
    // SERIAL_PC.print("] ");
    // SERIAL_PC.print("Estop: ");
    // SERIAL_PC.print(is_estop_rf_on);
    // SERIAL_PC.print(" | Drive Mode: ");
    // SERIAL_PC.print(drive_mode);
    // SERIAL_PC.print(" | state: ");
    // SERIAL_PC.print(state);
    // SERIAL_PC.print(" | Velocity: ");
    // SERIAL_PC.print(velocity);
    // SERIAL_PC.print(" | Omega : ");
    // SERIAL_PC.print(omega);
    // SERIAL_PC.print(" | Gear: ");
    // SERIAL_PC.print(gear);
    // SERIAL_PC.print(" | State: ");
    // SERIAL_PC.println(stateToString(state));

    // SERIAL_PC.print(is_estop);
    // SERIAL_PC.print(is_battery_low);
    // SERIAL_PC.print(is_estop_btn_on);
    // SERIAL_PC.print(is_estop_rf_on);
    // SERIAL_PC.print(is_rf_disconnected);
  }
}

// Motor control 
void taskRobotControl(){
  while(true){
    ef.wait_any(FLAG_ROBOT_CONTROL);
    
    // 4WS Robot
    int32_t l_rpm = 0;
    int32_t r_rpm = 0;

    float v;
    float w;
    float theta_l = 0.0;
    float theta_r = 0.0;

    zltech.loop();
    is_motor_connected = zltech.isConnected();

    // Common
    is_estop = (is_estop_btn_on || is_estop_rf_on || is_rf_disconnected || is_battery_low);

    if (! is_motor_connected) state = MOTOR_IDLE;
    else if (is_estop) state = ESTOP;

    switch (state) {
      case ESTOP: {
        if (!is_estop) {
          state = DRIVE_READY;
          break;
        }
        led_controller.estop();

        break;
      }
      
      case MOTOR_IDLE: {
        if (is_motor_connected) {
          state = DRIVE_READY;
        }
        
        // 4WS Robot
        zltech.setup();
        break;
      }

      case RF_IDLE: {
        if (!is_rf_disconnected){
          state = DRIVE_READY;
        }
        break;
      }

      case DRIVE_READY: {
        state = (drive_mode == MODE_MANUAL) ? DRIVE_MANUAL : AUTO_IDLE;
        break;
      }

      case DRIVE_MANUAL: {
        //============================  4WS Kinematics  ===============================
        if (drive_mode != MODE_MANUAL) {
          state = DRIVE_READY;
          break;
        }
        led_controller.driveManual();

        // velocity, omega
        // Motor Control
        v = (float)(velocity) / 100.0f; // -1 ~ 1[m/s]
        w = (float)(omega) / 1000.0f; // -1 ~ 1 [rad/s]  |  ← : (CCW), → : (CW)

        float v_left = sqrt((v - (THREAD / 2.0f) * w) * (v - (THREAD / 2.0f) * w) + ((w * WHEELBASE) / 2) * ((w * WHEELBASE) / 2));
        float v_right = sqrt((v + (THREAD / 2.0f) * w) * (v + (THREAD / 2.0f) * w) + ((w * WHEELBASE) / 2) * ((w * WHEELBASE) / 2));
        
        float rpm_left = v_left / WHEEL_R * 60.0f / (2.0f * PI);
        float rpm_right = v_right / WHEEL_R * 60.0f / (2.0f * PI);

        if (v >= 0) {
          l_rpm = -static_cast<int32_t>(rpm_left);
          r_rpm = -static_cast<int32_t>(rpm_right);
        }
        else if (v < 0) {
          l_rpm = static_cast<int32_t>(rpm_left);
          r_rpm = static_cast<int32_t>(rpm_right);
        }

        if (fabs(w) < 1e-6) {
            theta_l = 0.0f;
            theta_r = 0.0f;
        } else {
            theta_l = atan2(WHEELBASE, 2 * (fabs(v) / fabs(w) - (THREAD / 2.0f)));
            theta_r = atan2(WHEELBASE, 2 * (fabs(v) / fabs(w) + (THREAD / 2.0f)));
        }

        break;
      }

      case AUTO_IDLE: {
        if (drive_mode != MODE_AUTO) state = DRIVE_READY;

        state = (is_upper_connected) ? DRIVE_AUTO : AUTO_FAIL;

        break;
      }

      case DRIVE_AUTO: {
        if (drive_mode != MODE_AUTO) state = DRIVE_READY;

        break;
      }
      
      case AUTO_FAIL: {
        if (drive_mode != MODE_AUTO) state = DRIVE_READY;

        if (is_upper_connected) state = DRIVE_AUTO;
        break;
      }

      default: {
        break;
      }
    }
    // SERIAL_PC.println(l_rpm);
    // SERIAL_PC.println(-r_rpm);
    zltech.sendVelocity(l_rpm, -r_rpm);
    zltech.readVelocity(&left_actual_rpm, &right_actual_rpm);

    // Transceiver : ← (-) → (+)
    // Step Dir : CCW (-) CW (+)
    if (v != 0.0f && w < 0.0f) {  // Foward Turning Left or Backward Turning right
      stepmotor_1.sendAngle(theta_l);   // Real (CCW)  Step (CW)  FL
      stepmotor_2.sendAngle(-theta_l);    // Real (CW)  Step (CCW)  BL
      stepmotor_3.sendAngle(theta_r);   // Real (CCW)  Step (CW)  FR
      stepmotor_4.sendAngle(-theta_r);   // Real (CW)  Step (CCW)  BR
    }
    else if (v != 0.0f && w > 0.0f) {  // Foward Turning Right or Backward Turning Left
      stepmotor_1.sendAngle(-theta_l);   // Real (CW)  Step (CCW)  FL
      stepmotor_2.sendAngle(theta_l);  // Real (CCW)  Step (Cw)  BL
      stepmotor_3.sendAngle(-theta_r);   // Real (CW)  Step (CCW)  FR
      stepmotor_4.sendAngle(theta_r);   // Real (CCW)  Step (CW)  BR
    }
    else if (w == 0.0f) {
      stepmotor_1.sendAngle(0.0f);
      stepmotor_2.sendAngle(0.0f);
      stepmotor_3.sendAngle(0.0f);
      stepmotor_4.sendAngle(0.0f);
    }
    SERIAL_PC.println(theta_l);
    SERIAL_PC.println(theta_r);
  }
}

void taskUpperComm(){
  while(true){
    ef.wait_any(FLAG_UPPER_COMM);

    upper_comm.loop();
    upper_comm.sendPacket(drive_mode, estop, gear, velocity, omega, brake, left_actual_rpm, right_actual_rpm, battery);

    is_upper_connected = upper_comm.is_connected;
    
    if (is_upper_connected) {
      digitalWrite(LED_PIN, HIGH);  // 정상 데이터 수신 중이면 LED OFF
    } 
    else {
      if (millis() % 500 < 250) {
        digitalWrite(LED_PIN, HIGH);  // 0.25초 켜짐
      } else {
        digitalWrite(LED_PIN, LOW);   // 0.25초 꺼짐
      }
    }
  }
}

void taskBattery(){
  while(true){
    ef.wait_any(FLAG_BATTERY);

    uint16_t sensorValue = analogRead(voltagePin);
    battery = (sensorValue * 310UL) / 1023 - 100;

    is_battery_low = (battery < 120);
  }
}

void setFlag(){
  if(tick_cnt % tick_rf == 0) ef.set(FLAG_RF);
  if(tick_cnt % tick_robot_control == 0) ef.set(FLAG_ROBOT_CONTROL);
  if(tick_cnt % tick_upper == 0) ef.set(FLAG_UPPER_COMM);
  if(tick_cnt % tick_battery == 0) ef.set(FLAG_BATTERY);
  ++tick_cnt;
}

// void detectEstopSW() {
//     is_estop_btn_on = (digitalRead(PIN_ESTOP) == LOW);
// }

void loop() {
}

const char* stateToString(int state) {
    switch (state) {
        case ESTOP:       return "ESTOP";
        case MOTOR_IDLE:  return "MOTOR_IDLE";
        case RF_IDLE:     return "RF_IDLE";
        case DRIVE_READY: return "DRIVE_READY";
        case DRIVE_MANUAL:return "DRIVE_MANUAL";
        case AUTO_IDLE:   return "AUTO_IDLE";
        case DRIVE_AUTO:  return "DRIVE_AUTO";
        case AUTO_FAIL:   return "AUTO_FAIL";
        default:          return "UNKNOWN_STATE";
    }
}