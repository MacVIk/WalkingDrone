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

Terminal terminal;
DynamixelMX106 dxl(0xfe);

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

        if (uartTerm.uart_read_byte() == 5) {
            uint8_t okByte {0x01};
            uartTerm.uart_send(&okByte, sizeof(okByte));
            dxl.ping();
        } else {
            uint8_t errByte {0xff};
            uartTerm.uart_send(&errByte, sizeof(errByte));
        }

        /* Finish the task before next tick */
        taskYIELD();
    }
}

