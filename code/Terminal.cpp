/*
 * Terminal.cpp
 *
 *  Created on: 15 џэт. 2020 у.
 *      Author: Taras.Melnik
 */

#include <UartTerminal.h>
#include "Terminal.h"
#include "stm32f1xx_ll_usart.h"

Terminal terminal;

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

        uint8_t byte[5];

        for (uint8_t i = 0; i < 5; ++i)
            uartTerm.uart_read(byte, uartTerm.packSize);
        uartTerm.uart_send(byte, 5);

        /* Finish the task before next tick */
        taskYIELD();
    }
}

//extern "C" {
//void USART1_IRQHandler()
//{
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    vTaskNotifyGiveFromISR(terminal.taskHandle, &xHigherPriorityTaskWoken);
//    if (LL_USART_IsActiveFlag_RXNE(USART1)) {
//        LL_USART_ClearFlag_RXNE(USART1);
//        bufArr = LL_USART_ReceiveData8(USART1);
//    }
//    if (xHigherPriorityTaskWoken)
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}
//
//}
