/*
 * UartApi.h
 *
 *  Created on: 15 џэт. 2020 у.
 *      Author: Taras.Melnik
 *
 *      Template class for USART initialization
 *
 *
 */

#ifndef LIB_UARTAPI_H_
#define LIB_UARTAPI_H_

#include "stm32f1xx.h"

class UartApi {

public:
    UartApi();
    virtual ~UartApi();

    /*
     * Peripheral initialization,
     * use before to start RTOS scheduler
     */
    virtual void init() = 0;

    /* User interface */
    virtual void uart_read(uint8_t* arr, uint8_t len) = 0;
    virtual uint8_t uart_read_byte() = 0;
    virtual void uart_send(uint8_t* arr, uint8_t len) = 0;

protected:
    virtual void init_gpio() = 0;
    virtual void init_uart() = 0;
};

#endif /* LIB_UARTAPI_H_ */
