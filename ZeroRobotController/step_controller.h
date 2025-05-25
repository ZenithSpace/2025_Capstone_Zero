#ifndef CAN_STEP_CONTROLLER_H
#define CAN_STEP_CONTROLLER_H

#include <Arduino.h>
#include <Arduino_CAN.h>
#include <stdint.h>

class StepController {
    private:
        uint16_t speed = 700;     // range : 0 ~ 3000 | 700rpm : 0.2초에 1.2도 -> 기어비 1:14일 때 60도 회전하는데 약 0.2s
        uint8_t acc = 0;          // range : 0 ~ 255 | 0 : target speed 즉시 도달
        uint32_t send_pulse = 0;

    public:
        uint8_t _canID;

        StepController(uint8_t canID);                                             // canID 설정
        bool init();                                                               // CAN Bitrate 설정정
        void setGroupID(uint16_t groupID);                                         // groupID 설정 (50 또는 51)
        uint8_t calculateCRC(uint8_t canID, const uint8_t* data, size_t length);   // CHECKSUM 계산용 (CRC)
        void sendCWPulses();
        void sendCCWPulses();
        void sendAngle(float angle_rad);
        void readEncoderValue();
        void checkEncoderResponse();
};

#endif