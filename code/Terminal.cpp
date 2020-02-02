/*
 * Terminal.cpp
 *
 *  Created on: 15 џэт. 2020 у.
 *      Author: Taras.Melnik
 */

#include <UartTerminal.h>
#include "Terminal.h"
#include "stm32f1xx_ll_usart.h"
#include "DynamixelMX106.h"
#include "DynamixelProtocol_1_0.h"
#include "UartDynamixel.h"
#include <string.h>

using namespace DynamixelProtocol_1_0;

static const uint8_t DXL_ID = 0xfe;

Terminal terminal;
DynamixelMX106 dxl(DXL_ID);
UartDynamixel* uartDxl = UartDynamixel::getInstance();

static uint8_t arr_tx[20];
static uint8_t arr_rx[20];

enum {
    PING = 5,
    READ_ANGLE,
    SET_ANGLE,
    DXL_READ_ANGLE,
    DXL_SET_ANGLE
};

Terminal::Terminal() {
    // TODO Auto-generated constructor stub

}

Terminal::~Terminal() {
    // TODO Auto-generated destructor stub
}

void Terminal::run() {

    uartTerm.set_notifucation(this->taskHandle);

    while (1) {
        /* Task is suspended until notification */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //First debug stage
        if (uartTerm.uart_read_byte() == PING) {
            uint8_t len = makePingPacket(DXL_ID, arr_tx);
            uartTerm.uart_send(arr_tx, len);
            uartDxl->uart_send(arr_tx, len);
            while(!(uartDxl->packReadyFlag));
            unpack(arr_rx);
            uartTerm.uart_send(arr_rx, DXL_INSTRUCTION_PING_SIZE);

        } else if (uartTerm.uart_read_byte() == READ_ANGLE) {
            uint8_t len = makeReadWordPacket(DXL_ID, arr_tx, MX106_REG_PRESENT_POSITION);
            uartTerm.uart_send(arr_tx, len);
            uartDxl->uart_send(arr_tx, len);
            while(!(uartDxl->packReadyFlag));
            unpack(arr_rx);
            uartTerm.uart_send(arr_rx, DXL_INSTRUCTION_PING_SIZE);

        } else if (uartTerm.uart_read_byte() == SET_ANGLE) {
            uint16_t word;
            uartTerm.uart_read(arr_rx, 3);
            memcpy(&word, &arr_rx, sizeof(word));
            uint8_t len = makeWriteWordPacket(DXL_ID, arr_tx, MX106_REG_GOAL_POSITION, word);
            uartTerm.uart_send(arr_tx, len);
            uartDxl->uart_send(arr_tx, len);
            while(!(uartDxl->packReadyFlag));
            unpack(arr_rx);
            uartTerm.uart_send(arr_rx, DXL_INSTRUCTION_PING_SIZE);

            //Second debug stage
        } else if (uartTerm.uart_read_byte() == DXL_SET_ANGLE) {
            uint16_t position = 0;
            uint16_t error;
            arr_tx[0] = DXL_SET_ANGLE;
            uartTerm.uart_send(arr_tx, 1);

            error = dxl.readAngle(position);

            // Error
        } else {
            uint8_t errror[] {0xfe, 0xff};
            uartTerm.uart_send(errror, sizeof(errror));
        }
        /* Finish the task before next tick */
        taskYIELD();
    }
}

