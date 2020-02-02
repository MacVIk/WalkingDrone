/*
 * DynamixelMX106.h
 *
 *  Created on: 30 jan. 2020 ã.
 *      Author: Taras.Melnik
 *
 *  Based on datasheet for MX106 and protocol 1.0
 */

#ifndef LIB_DYNAMIXELMX106_H_
#define LIB_DYNAMIXELMX106_H_

// Buffer sizes
#define MX106_MAX_PACKET_SIZE_TX        20
#define MX106_MAX_PACKET_SIZE_RX        20

// MX106 registers
// Flash registers
#define MX106_REG_DXL_ID                3

// RAM registers
#define MX106_REG_GOAL_POSITION         30
#define MX106_REG_PRESENT_POSITION      36
#define MX106_REG_MOVING                46
#define MX106_REG_TORQUE_ENABLE         24
#define MX106_REG_PRESENT_LOAD          40
#define MX106_REG_PRESENT_VOLTAGE       42

// Internal flags
#define MX106_NO_AKNOWLEGE              ((uint16_t) 0b00000010)
#define MX106_PACKET_INVALID            ((uint16_t) 0b00000001)


#include <stdint.h>

#include "DynamixelProtocol_1_0.h"

typedef DynamixelProtocol_1_0::UnpackResult UnpackRes;

class DynamixelMX106 {
public:
    DynamixelMX106(uint8_t id);
    virtual ~DynamixelMX106() = default;

    /* Primary dxl settings */

    /* Communication */
    /* Write */
    uint16_t ping();
    uint16_t setAngle(uint16_t position);
    uint16_t setTorque(bool position);

    /* Read */
    uint16_t readAngle(uint16_t &pos);
    uint16_t readMovStatus(uint16_t &movFlag);
    uint16_t readTorque(uint16_t &torq);
    uint16_t readVoltage(uint16_t &volt);

private:
    uint8_t sendArr[MX106_MAX_PACKET_SIZE_TX];
    uint8_t readArr[MX106_MAX_PACKET_SIZE_RX];

    uint8_t id;
    uint8_t error;

    uint16_t sendPack(uint8_t len);
    uint16_t processPacket(UnpackRes result);

};

#endif /* LIB_DYNAMIXELMX106_H_ */
