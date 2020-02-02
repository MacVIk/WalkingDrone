/*
 * DynamixelMX106.cpp
 *
 *  Created on: 30 џэт. 2020 у.
 *      Author: Taras.Melnik
 */

#include "DynamixelMX106.h"
#include "UartDynamixel.h"

using namespace DynamixelProtocol_1_0;

// Use USART2 peripheral
UartDynamixel* uart = UartDynamixel::getInstance();

DynamixelMX106::DynamixelMX106(uint8_t id) {
    this->id = id;
    this->error = 0;
}

uint16_t DynamixelMX106::sendPack(uint8_t len)
{
    uart->uart_send(sendArr, len);
    while (!(uart->packReadyFlag)) {
        //ToDo
        return error |= MX106_NO_AKNOWLEGE << 8;
    }
    uart->uart_read(readArr, MX106_MAX_PACKET_SIZE_RX);
    return 0;
}

uint16_t DynamixelMX106::processPacket(UnpackRes result)
{
    //ToDo invalid packet;
    error = result.errByte;
    result.packetValid ? (error &= 0xff) : (error |= MX106_PACKET_INVALID << 8);
    return result.data;
}

uint16_t DynamixelMX106::ping()
{
    sendPack(makePingPacket(id, sendArr));
    processPacket(unpack(readArr));
    return error;
}

uint16_t DynamixelMX106::setAngle(uint16_t position)
{
    sendPack(makeWriteWordPacket(id, sendArr, MX106_REG_GOAL_POSITION, position));
    processPacket(unpack(readArr));
    return error;
}

uint16_t DynamixelMX106::setTorque(bool position)
{
    sendPack(makeWriteWordPacket(id, sendArr, MX106_REG_GOAL_POSITION, position));
    processPacket(unpack(readArr));
    return error;
}

uint16_t DynamixelMX106::readAngle(uint16_t &pos)
{
    sendPack(makeReadWordPacket(id, sendArr, MX106_REG_PRESENT_POSITION));
    pos = processPacket(unpack(readArr));
    return error;
}

uint16_t DynamixelMX106::readMovStatus(uint16_t &movFlag)
{
    sendPack(makeReadBytePacket(id, sendArr, MX106_REG_MOVING));
    movFlag = processPacket(unpack(readArr));
    return error;
}

uint16_t DynamixelMX106::readTorque(uint16_t &torq)
{
    sendPack(makeReadBytePacket(id, sendArr, MX106_REG_PRESENT_LOAD));
    torq = processPacket(unpack(readArr));
    return error;
}

uint16_t DynamixelMX106::readVoltage(uint16_t &volt)
{
    sendPack(makeReadBytePacket(id, sendArr, MX106_REG_PRESENT_VOLTAGE));
    volt = processPacket(unpack(readArr));
    return error;
}



