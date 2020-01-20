/*
 * UartDriver.h
 *
 *  Created on: 15 џэт. 2020 у.
 *      Author: Taras.Melnik
 */

#ifndef LIB_UARTDRIVER_H_
#define LIB_UARTDRIVER_H_

#include "UartApi.h"
#include "FreeRTOSConfig.h"

/*Uart configuration options */
#define TERM_UART_BAUDRATE                      115200
#define TERM_UART_DATA_WIDTH                    LL_USART_DATAWIDTH_8B
#define TERM_UART_HAWDWARE_FLOAT_CNTRL          LL_USART_HWCONTROL_NONE
#define TERM_UART_PARITY                        LL_USART_PARITY_NONE
#define TERM_UART_STOPBITS                      LL_USART_STOPBITS_1
#define TERM_UART_TRANSFER_DIRECTION            LL_USART_DIRECTION_TX_RX
#define TERM_UART_IRQN_PRIORITY                 (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)
#define TERM_UART_IRQN_RX                       USART1_IRQn

/* DMA configuration macros */
#define TERM_DMA_SRC_ADDR_RX                    (uint32_t)&((TERM_UART)->DR)
#define TERM_DMA_DIRECTION_RX                   LL_DMA_DIRECTION_PERIPH_TO_MEMORY
#define TERM_DMA_MODE_RX                        LL_DMA_MODE_NORMAL
#define TERM_DMA_PERIPH_INC_MODE_RX             LL_DMA_PERIPH_NOINCREMENT
#define TERM_DMA_MEM_INC_MODE_RX                LL_DMA_MEMORY_INCREMENT
#define TERM_DMA_PERIPH_SIZE_RX                 LL_DMA_PDATAALIGN_BYTE
#define TERM_DMA_MEM_SIZE_RX                    LL_DMA_MDATAALIGN_BYTE
#define TERM_DMA_BUFFER_SIZE_RX                 ((uint32_t) 64)
#define TERM_DMA_PRIORITY_RX                    LL_DMA_PRIORITY_HIGH
#define TERM_DMA_IRQN_PRIORITY_RX               (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)


#define TERM_DMA_DST_ADDR_TX                    (uint32_t)&((TERM_UART)->DR)
#define TERM_DMA_DIRECTION_TX                   LL_DMA_DIRECTION_MEMORY_TO_PERIPH
#define TERM_DMA_MODE_TX                        LL_DMA_MODE_NORMAL
#define TERM_DMA_PERIPH_INC_MODE_TX             LL_DMA_PERIPH_NOINCREMENT
#define TERM_DMA_MEM_INC_MODE_TX                LL_DMA_MEMORY_INCREMENT
#define TERM_DMA_MEM_SIZE_TX                    LL_DMA_MDATAALIGN_BYTE
#define TERM_DMA_PERIPH_SIZE_TX                 LL_DMA_PDATAALIGN_BYTE
#define TERM_DMA_PRIORITY_TX                    LL_DMA_PRIORITY_HIGH
#define TERM_DMA_BUFFER_SIZE_TX                 ((uint32_t) 64)
#define TERM_DMA_IRQN_PRIORITY_TX               (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)


class UartDriver : public UartApi {
public:
    UartDriver();
    virtual ~UartDriver();

    void init() override;

    void uart_read(uint8_t* arr, uint8_t len) override;
    uint8_t uart_read_byte() override;
    void uart_send(uint8_t* arr, uint8_t len) override;
    //todo bad solution
    void set_notifucation(TaskHandle_t taskHandle);
    TaskHandle_t get_notification();

private:
    void init_gpio() override;
    void init_uart() override;
    void init_dma();

    uint8_t byteN;
    uint8_t readArr[TERM_DMA_BUFFER_SIZE_RX];
    uint8_t sendArr[TERM_DMA_BUFFER_SIZE_TX];
    TaskHandle_t taskHandle;
};

extern UartDriver uartTerm;

#endif /* LIB_UARTDRIVER_H_ */
