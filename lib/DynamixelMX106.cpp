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
    this->position = 0;
}

void DynamixelMX106::sendPack(uint8_t len)
{
    uart->uart_send(sendArr, len);
    while (!(uart->packReadyFlag)) {
        //ToDo
    }
    uart->uart_read(readArr, MX106_MAX_PACKET_SIZE_RX);
}

uint16_t DynamixelMX106::processPacket(UnpackRes result)
{
    //ToDo invalid packet;
    error = result.errByte;
    return result.data;
}

void DynamixelMX106::ping()
{
    sendPack(makePingPacket(id, sendArr));
    processPacket(unpack(readArr));
}

void DynamixelMX106::setAngle(uint16_t position)
{
    sendPack(makeWriteWordPacket(id, sendArr, MX106_REG_GOAL_POSITION, position));
    processPacket(unpack(readArr));
}

uint16_t DynamixelMX106::readAngle()
{
    sendPack(makeReadWordPacket(id, sendArr, MX106_REG_PRESENT_POSITION));
    return processPacket(unpack(readArr));
}

uint16_t DynamixelMX106::movStatus()
{
    sendPack(makeReadBytePacket(id, sendArr, MX106_REG_MOVING));
    return processPacket(unpack(readArr));
}




