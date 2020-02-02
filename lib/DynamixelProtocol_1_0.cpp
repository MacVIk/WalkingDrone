/*
 * DynamixelProtocol_1_0.cpp
 *
 *  Created on: 2 февр. 2020 г.
 *      Author: Taras.Melnik
 */

#include "DynamixelProtocol_1_0.h"

namespace DynamixelProtocol_1_0 {

#include <stdio.h>

static struct lastRequest{
    uint8_t id;
    uint8_t instruction;
    uint8_t bytes2Read;
} lastReq;

inline uint8_t crcCaclculate(uint8_t* dataArr, uint8_t length) {
    uint8_t crc {0};
    for (uint8_t i = DXL_HEADER_SIZE; i < length; ++i)
        crc += dataArr[i];
    return ~crc;
}

inline uint8_t lowByte(uint16_t word) {
    return word & 0xFF;
}

inline uint8_t highByte(uint16_t word) {
    return (word >> 8) & 0xFF;
}

// Returns the full packet size.
uint16_t makePingPacket(uint8_t id, uint8_t* dataArr)
{
    DXL_PREFIX_CREATE();
    dataArr[3] = DXL_INSTRUCTION_PING_LENGTH;
    dataArr[4] = DXL_INSTRUCTION_PING;
    uint8_t crcByte = DXL_INSTRUCTION_PING_SIZE - 1;
    dataArr[crcByte] = crcCaclculate(dataArr, crcByte);
    return DXL_INSTRUCTION_PING_SIZE;
}

uint16_t makeReadBytePacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr)
{
    DXL_PREFIX_CREATE();
    dataArr[3] = DXL_INSTRUCTION_READ_LENGTH;
    dataArr[4] = DXL_INSTRUCTION_READ;
    dataArr[5] = startAdrr;
    dataArr[6] = sizeof(uint8_t);
    uint8_t crcByte = DXL_INSTRUCTION_READ_SIZE - 1;
    dataArr[crcByte] = crcCaclculate(dataArr, crcByte);
    return DXL_INSTRUCTION_READ_SIZE;
}

uint16_t makeReadWordPacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr)
{
    DXL_PREFIX_CREATE();
    dataArr[3] = DXL_INSTRUCTION_READ_LENGTH;
    dataArr[4] = DXL_INSTRUCTION_READ;
    dataArr[5] = startAdrr;
    dataArr[6] = sizeof(uint16_t);
    uint8_t crcByte = DXL_INSTRUCTION_READ_SIZE - 1;
    dataArr[crcByte] = crcCaclculate(dataArr, crcByte);
    return DXL_INSTRUCTION_READ_SIZE;
}

uint16_t makeWriteBytePacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr, uint8_t byte)
{
    DXL_PREFIX_CREATE();
    dataArr[3] = DXL_INSTRUCTION_WRITE_BYTE_LENGTH;
    dataArr[4] = DXL_INSTRUCTION_WRITE;
    dataArr[5] = startAdrr;
    dataArr[6] = byte;
    uint8_t crcByte = DXL_INSTRUCTION_WRITE_BYTE_SIZE - 1;
    dataArr[crcByte] = crcCaclculate(dataArr, crcByte);
    return DXL_INSTRUCTION_WRITE_BYTE_SIZE;
}

uint16_t makeWriteWordPacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr, uint16_t word)
{
    DXL_PREFIX_CREATE();
    dataArr[3] = DXL_INSTRUCTION_WRITE_WORD_LENGTH;
    dataArr[4] = DXL_INSTRUCTION_WRITE;
    dataArr[5] = startAdrr;
    dataArr[6] = lowByte(word);
    dataArr[7] = highByte(word);
    uint8_t crcByte = DXL_INSTRUCTION_WRITE_WORD_SIZE - 1;
    dataArr[crcByte] = crcCaclculate(dataArr, crcByte);
    return DXL_INSTRUCTION_WRITE_WORD_SIZE;
}

uint16_t makeRebootPacket(uint8_t id, uint8_t* dataArr)
{
    DXL_PREFIX_CREATE();
    dataArr[3] = DXL_INSTRUCTION_REBOOT_LENGTH;
    dataArr[4] = DXL_INSTRUCTION_REBOOT;
    uint8_t crcByte = DXL_INSTRUCTION_REBOOT_SIZE - 1;
    dataArr[crcByte] = crcCaclculate(dataArr, crcByte);
    return DXL_INSTRUCTION_REBOOT_SIZE;
}

UnpackResult unpack(uint8_t* dataArr)
{
    UnpackResult result;
    result.packetValid = true;
    result.errByte = dataArr[4];
    result.data = 0;

    uint8_t packetSize = dataArr[4] + 3;
    uint8_t crc {0};

    for (uint8_t i = DXL_HEADER_SIZE; i <= packetSize; ++i) {
        crc += dataArr[i];
    }
    if (crc == dataArr[packetSize]) {
        if (result.errByte != 0) {
            return result;
        }
        if (dataArr[3] == DXL_PARAMETR_SIZE_BYTE) {
            result.data = dataArr[5];
        } else if (dataArr[3] == DXL_PARAMETR_SIZE_WORD) {
            result.data = dataArr[5];
            result.data |= (((uint16_t) dataArr[6]) << 8);
        }
        return result;
    } else {
        result.packetValid = false;
        return result;
    }
}

} /* namespace DynamixelProtocol_1_0 */
