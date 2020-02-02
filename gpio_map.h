/*
 * gpio_map.h
 *
 *  Created on: 15 џэт. 2020 у.
 *      Author: Taras.Melnik
 *
 *      This file describes all GPIO with appropriate interfaces.
 *      More detailed information about alternative functions
 *      see in af_map.h file.
 *
 */

#ifndef GPIO_MAP_H_
#define GPIO_MAP_H_

#include "af_map.h"

/* TERM_UART */

#define TERM_UART_PORT              GPIOA
#define TERM_UART_PIN_TX            LL_GPIO_PIN_9
#define TERM_UART_OTPUT_TYPE_TX     LL_GPIO_OUTPUT_PUSHPULL
#define TERM_UART_PIN_RX            LL_GPIO_PIN_10
#define TERM_UART_OTPUT_TYPE_RX     LL_GPIO_MODE_FLOATING

/* DXL_UART */
#define DXL_UART_PORT               GPIOA
#define DXL_UART_PIN_RX_TX          LL_GPIO_PIN_2
#define DXL_UART_PIN_MODE_TX        LL_GPIO_MODE_ALTERNATE
#define DXL_UART_PIN_MODE_RX        LL_GPIO_MODE_FLOATING


#endif /* GPIO_MAP_H_ */
