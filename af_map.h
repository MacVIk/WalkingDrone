/*
 * af_map.h
 *
 *  Created on: 15 џэт. 2020 у.
 *      Author: Taras.Melnik
 */

#ifndef AF_MAP_H_
#define AF_MAP_H_

/*
 * Terminal configuration
 */
/* UART */
#define TERM_UART                               USART1

/* DMA */
#define TERM_DMA                                DMA1
#define TERM_DMA_CHANNEL_RX                     LL_DMA_CHANNEL_5
#define TERM_DMA_IRQN_RX                        DMA1_Channel5_IRQn
#define TERM_DMA_CHANNEL_TX                     LL_DMA_CHANNEL_4
#define TERM_DMA_IRQN_TX                        DMA1_Channel4_IRQn

/*
 * Dynamixel configuration
 */
 /* UART */
#define DXL_UART                                USART2
#define DXL_UART_IRQN_RX                        USART1_IRQn

/* DMA */
#define DXL_DMA                                 DMA1
#define DXL_DMA_CHANNEL_TX                      LL_DMA_CHANNEL_7
#define DXL_DMA_IRQN_TX                         DMA1_Channel7_IRQn

#endif /* AF_MAP_H_ */
