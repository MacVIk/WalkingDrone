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
 *
 * UART
  */
#define TERM_UART                               USART1
#define TERM_UART_BAUDRATE                      115200
#define TERM_UART_DATA_WIDTH                    LL_USART_DATAWIDTH_8B
#define TERM_UART_HAWDWARE_FLOAT_CNTRL          LL_USART_HWCONTROL_NONE
#define TERM_UART_PARITY                        LL_USART_PARITY_NONE
#define TERM_UART_STOPBITS                      LL_USART_STOPBITS_1
#define TERM_UART_TRANSFER_DIRECTION            LL_USART_DIRECTION_TX_RX
#define TERM_UART_IRQN_PRIORITY                 (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)
#define TERM_UART_IRQN_RX                       USART1_IRQn

/* DMA */

#define TERM_DMA                                DMA1

#define TERM_DMA_CHANNEL_RX                     LL_DMA_CHANNEL_5
#define TERM_DMA_IRQN_RX                        DMA1_Channel5_IRQn
#define TERM_DMA_MODE_RX                        LL_DMA_MODE_NORMAL
#define TERM_DMA_DIRECTION_RX                   LL_DMA_DIRECTION_PERIPH_TO_MEMORY
#define TERM_DMA_PRIORITY_RX                    LL_DMA_PRIORITY_HIGH
#define TERM_DMA_MEM_INC_MODE_RX                LL_DMA_MEMORY_INCREMENT
#define TERM_DMA_MEM_SIZE_RX                    LL_DMA_MDATAALIGN_BYTE
#define TERM_DMA_PERIPH_INC_MODE_RX             LL_DMA_PERIPH_NOINCREMENT
#define TERM_DMA_PERIPH_SIZE_RX                 LL_DMA_PDATAALIGN_BYTE
#define TERM_DMA_SRC_ADDR_RX                    (uint32_t)&((TERM_UART)->DR)
#define TERM_DMA_IRQN_PRIORITY_RX               ((uint32_t) 5)

#define TERM_DMA_CHANNEL_TX                     LL_DMA_CHANNEL_4
#define TERM_DMA_IRQN_TX                        DMA1_Channel4_IRQn
#define TERM_DMA_MODE_TX                        LL_DMA_MODE_NORMAL
#define TERM_DMA_DIRECTION_TX                   LL_DMA_DIRECTION_PERIPH_TO_MEMORY
#define TERM_DMA_PRIORITY_TX                    LL_DMA_PRIORITY_HIGH
#define TERM_DMA_MEM_INC_MODE_TX                LL_DMA_MEMORY_INCREMENT
#define TERM_DMA_MEM_SIZE_TX                    LL_DMA_MDATAALIGN_BYTE
#define TERM_DMA_PERIPH_INC_MODE_TX             LL_DMA_PERIPH_NOINCREMENT
#define TERM_DMA_PERIPH_SIZE_TX                 LL_DMA_PDATAALIGN_BYTE
#define TERM_DMA_SRC_ADDR_TX                    (uint32_t)&((TERM_UART)->DR)
#define TERM_DMA_IRQN_PRIORITY_TX               ((uint32_t) 5)



#endif /* AF_MAP_H_ */
