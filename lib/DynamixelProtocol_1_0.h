/*
 * DynamixelProtocol_1_0.h
 *
 *  Created on: 2 февр. 2020 г.
 *      Author: Taras.Melnik
 */

#ifndef LIB_DYNAMIXELPROTOCOL_1_0_H_
#define LIB_DYNAMIXELPROTOCOL_1_0_H_

namespace DynamixelProtocol_1_0 {

#include <stdint.h>

//Instruction definition
#define DXL_INSTRUCTION_PING                ((uint8_t) 0x01)
#define DXL_INSTRUCTION_READ                ((uint8_t) 0x02)
#define DXL_INSTRUCTION_WRITE               ((uint8_t) 0x03)
#define DXL_INSTRUCTION_REG_WRITE           ((uint8_t) 0x04)
#define DXL_INSTRUCTION_ACTION              ((uint8_t) 0x05)
#define DXL_INSTRUCTION_FACTORY_RESET       ((uint8_t) 0x06)
#define DXL_INSTRUCTION_REBOOT              ((uint8_t) 0x08)
#define DXL_INSTRUCTION_SYNC_WRITE          ((uint8_t) 0x083)
#define DXL_INSTRUCTION_BULK_READ           ((uint8_t) 0x092)

// Packet length
#define DXL_INSTRUCTION_PING_LENGTH         ((uint8_t) 2)
#define DXL_INSTRUCTION_READ_LENGTH         ((uint8_t) 4)
#define DXL_INSTRUCTION_WRITE_BYTE_LENGTH   ((uint8_t) 4)
#define DXL_INSTRUCTION_WRITE_WORD_LENGTH   ((uint8_t) 5)
#define DXL_INSTRUCTION_REG_WRITE_LENGTH    ((uint8_t) 5)
#define DXL_INSTRUCTION_ACTION_LENGTH       ((uint8_t) 2)
#define DXL_INSTRUCTION_FACTORY_RESET_LENGTH  ((uint8_t) 2)
#define DXL_INSTRUCTION_REBOOT_LENGTH       ((uint8_t) 2)

// The full packet size (bytes number)
#define DXL_INSTRUCTION_PING_SIZE           ((uint8_t) 6)
#define DXL_INSTRUCTION_READ_SIZE           ((uint8_t) 8)
#define DXL_INSTRUCTION_WRITE_BYTE_SIZE     ((uint8_t) 8)
#define DXL_INSTRUCTION_WRITE_WORD_SIZE     ((uint8_t) 9)
#define DXL_INSTRUCTION_REG_WRITE_SIZE      ((uint8_t) 9)
#define DXL_INSTRUCTION_ACTION_SIZE         ((uint8_t) 6)
#define DXL_INSTRUCTION_FACTORY_RESET_SIZE  ((uint8_t) 6)
#define DXL_INSTRUCTION_REBOOT_SIZE         ((uint8_t) 6)

// Packet message
#define DXL_HEADER_SIZE                     ((uint8_t) 2)
#define DXL_PREFIX_SIZE                     ((uint8_t) 5)
#define CHECKSUM_SIZE                       ((uint8_t) 1)

//Unpack message
#define DXL_PARAMETR_SIZE_BYTE              ((uint8_t) 3)
#define DXL_PARAMETR_SIZE_WORD              ((uint8_t) 4)

// Prefix create function
#define DXL_PREFIX_CREATE() \
        dataArr[0] = 0xff; \
        dataArr[1] = 0xff; \
        dataArr[2] = id;

#define DXL_SAVE_REQUEST() \
        lastReq.id = id; \
        lastReq.instruction = dataArr[4]; \
        lastReq.bytes2Read = 1;             // Except makeReadWordPacket

// Unpack result
struct UnpackResult {
    uint16_t data;
    uint8_t errByte;
    bool packetValid;
};

// Debug functions
void PrintPacket(uint8_t* data, uint16_t num_bytes);

// Pack functions
uint16_t makePingPacket(uint8_t id, uint8_t* dataArr);
uint16_t makeReadBytePacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr);
uint16_t makeReadWordPacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr);
uint16_t makeWriteBytePacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr, uint16_t byte);
uint16_t makeWriteWordPacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr, uint16_t word);
uint16_t makeRebootPacket(uint8_t id, uint8_t* dataArr);

// Unpack functions
UnpackResult unpack(uint8_t* dataArr);

/* ToDo
 *
uint16_t makeReadPacket(uint8_t id, uint8_t* dataArr, uint8_t startAdrr, uint8_t bytesNumb);

uint16_t makeRegWritePacket(uint8_t id, uint8_t* dataArr);
uint16_t makeActionPacket(uint8_t id, uint8_t* dataArr);
uint16_t makeFactoryResetPacket(uint8_t id, uint8_t* dataArr);
uint16_t makeSincWritePacket(uint8_t id, uint8_t* dataArr);
uint16_t makeBulkReadPacket(uint8_t id, uint8_t* dataArr);
*/

} /* namespace DynamixelProtocol_1_0 */

#endif /* LIB_DYNAMIXELPROTOCOL_1_0_H_ */
