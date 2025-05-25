#include "step_controller.h"
#include <math.h>

// ========== CAN ID 설정 ==========
StepController::StepController(uint8_t canID)
    : _canID(canID) {}

// ========== CAN BitRate 설정 ==========
bool StepController::init() {
    if (!CAN.begin(CanBitRate::BR_500k)) {
        Serial.println("CAN 초기화 실패");
        while(1);
        return false;
    }
    return true;
}

// ========== CHECKSUM 계산 ==========
uint8_t StepController::calculateCRC(uint8_t canID, const uint8_t* data, size_t length) {
    uint8_t crc = canID;    // CRC = (ID + byte1 + ~ + byte(n)) & 0xFF
    for (size_t i = 0; i < length; ++i) {
      crc += data[i];
    }
    return crc & 0xFF;
}

// ========== groupID 설정 ==========
void StepController::setGroupID(uint16_t groupID) {
    uint8_t dataFrame[4] = {0};

    dataFrame[0] = 0x8D;                                  // byte1 (code) | groupID
    dataFrame[1] = (groupID >> 8) & 0x07;                 // byte2 (상위 3비트만 유효)
    dataFrame[2] = groupID & 0xFF;                        // byte3 (하위 8비트)
    dataFrame[3] = calculateCRC(_canID, dataFrame, 3);    // byte4 (CRC)

    // CAN 메시지 전송
    if (CAN.write(CanMsg(CanStandardId(_canID), 4, dataFrame))) {
        Serial.println("Group ID 설정 성공");
    }
    delay(10);
}

// ========== CW Pulses 전송 ==========
void StepController::sendCWPulses() {
    uint8_t dataFrame[8] = {0};                              // data : 8bytes

    dataFrame[0] = 0xFD;                                     // byte1 (code) | position model
    dataFrame[1] = 0x80;                                     // byte2-3 (dir & speed) | 0x80 : CW
    dataFrame[1] |= (speed >> 8) & 0x0F;                     // 상위 4비트 저장 (0x0F 마스크)
    dataFrame[2] = speed & 0xFF;                             // 하위 8비트 저장
    dataFrame[3] = acc;                                      // byte4 (acc)
    dataFrame[4] = (send_pulse >> 16) & 0xFF;                // byte5-7 (pulses)
    dataFrame[5] = (send_pulse >> 8) & 0xFF;
    dataFrame[6] = send_pulse & 0xFF;
    dataFrame[7] = calculateCRC(_canID, dataFrame, 7);       // byte8 (CRC)

    if (!CAN.write(CanMsg(CanStandardId(_canID), 8, dataFrame))) {
        Serial.println("CW Pulses 전송 실패");
    }
    delay(10);
}

// ========== CCW Pulses 전송 ==========
void StepController::sendCCWPulses() {
    uint8_t dataFrame[8] = {0};                              // data : 8bytes

    dataFrame[0] = 0xFD;                                     // byte1 (code) | position model
    dataFrame[1] = 0x00;                                     // byte2-3 (dir & speed) | 0x00 : CCW
    dataFrame[1] |= (speed >> 8) & 0x0F;                     // 상위 4비트 저장 (0x0F 마스크)
    dataFrame[2] = speed & 0xFF;                             // 하위 8비트 저장
    dataFrame[3] = acc;                                      // byte4 (acc)
    dataFrame[4] = (send_pulse >> 16) & 0xFF;                // byte5-7 (pulses)
    dataFrame[5] = (send_pulse >> 8) & 0xFF;
    dataFrame[6] = send_pulse & 0xFF;
    dataFrame[7] = calculateCRC(_canID, dataFrame, 7);       // byte8 (CRC)

    if (!CAN.write(CanMsg(CanStandardId(_canID), 8, dataFrame))) {
        Serial.println("CCW Pulses 전송 실패");
    }
    delay(10);
}

// ========== pulse 계산 ==========
void StepController::sendAngle(float angle_rad) {
    if (fabs(angle_rad) < 0.01f) return;
    else {
        float angle_deg = abs(angle_rad) * (180.0 / M_PI);    // for debuging
        // float angle_deg = abs(angle_rad) * (180.0 / M_PI) * 14;
        send_pulse = round(angle_deg / 0.1125);      // 0.1125 = 360 / (200 * MStep) and current MStep is 16

        if (angle_rad >= 0.0f) {
            sendCWPulses();
        }
        else if (angle_rad < 0.0f) {
            sendCCWPulses();
        }
    }
}



void StepController::readEncoderValue() {
    uint8_t dataFrame[2] = {0};

    dataFrame[0] = 0x39; // command
    dataFrame[1] = calculateCRC(_canID, dataFrame, 1);

    if (CAN.write(CanMsg(CanStandardId(_canID), 2, dataFrame))) {
        
    }
    delay(10); // 응답 대기
    checkEncoderResponse(); // 응답 처리
    delay(10);
}

void StepController::checkEncoderResponse() {

    if (CAN.available()) {
        

        CanMsg msg = CAN.read();


        if (msg.id == _canID && msg.data[0] == 0x39) {

            uint8_t receivedCRC = msg.data[5];
            uint8_t calculatedCRC = calculateCRC(msg.id, msg.data, 5);


            if (calculatedCRC == receivedCRC) {

                int32_t errorRaw = 0;
                errorRaw |= ((int32_t)msg.data[1]) << 24;
                errorRaw |= ((int32_t)msg.data[2]) << 16;
                errorRaw |= ((int32_t)msg.data[3]) << 8;
                errorRaw |= ((int32_t)msg.data[4]);

                float errorDegree = (float)errorRaw * 360.0f / 51200.0f;

                Serial.print("Error (degree): ");
                Serial.println(errorDegree);
            } else {
                Serial.println("CRC mismatch");
            }
        }
    }
}





