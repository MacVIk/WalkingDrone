/*
 * UartDynamixel.h
 *
 *  Created on: 19 jan. 2020 ã.
 *      Author: Taras.Melnik
 */

#ifndef LIB_UARTDYNAMIXEL_H_
#define LIB_UARTDYNAMIXEL_H_

#include "UartApi.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"

/*Uart configuration options */
#define DXL_UART_BUFFER_SIZE_TX                ((uint32_t) 64)
#define DXL_UART_BUFFER_SIZE_RX                ((uint32_t) 64)
#define DXL_UART_BAUDRATE                      ((uint32_t) 1000000)
#define DXL_UART_DATA_WIDTH                    LL_USART_DATAWIDTH_8B
#define DXL_UART_HAWDWARE_FLOAT_CNTRL          LL_USART_HWCONTROL_NONE
#define DXL_UART_PARITY                        LL_USART_PARITY_NONE
#define DXL_UART_STOPBITS                      LL_USART_STOPBITS_1
#define DXL_UART_TRANSFER_DIRECTION            LL_USART_DIRECTION_TX_RX
#define DXL_UART_IRQN_PRIORITY                 (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)

/*Dma configuration options */
#define DXL_DMA_DST_ADDR_TX                    (uint32_t)&((DXL_UART)->DR)
#define DXL_DMA_DIRECTION_TX                   LL_DMA_DIRECTION_MEMORY_TO_PERIPH
#define DXL_DMA_MODE_TX                        LL_DMA_MODE_NORMAL
#define DXL_DMA_PERIPH_INC_MODE_TX             LL_DMA_PERIPH_NOINCREMENT
#define DXL_DMA_MEM_INC_MODE_TX                LL_DMA_MEMORY_INCREMENT
#define DXL_DMA_MEM_SIZE_TX                    LL_DMA_MDATAALIGN_BYTE
#define DXL_DMA_PERIPH_SIZE_TX                 LL_DMA_PDATAALIGN_BYTE
#define DXL_DMA_PRIORITY_TX                    LL_DMA_PRIORITY_HIGH
#define DXL_DMA_BUFFER_SIZE_TX                 DXL_UART_BUFFER_SIZE_TX
#define DXL_DMA_IRQN_PRIORITY_TX               (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1)

class UartDynamixel: public UartApi {
public:
    static UartDynamixel* getInstance();
    void init() override;
    virtual void uart_read(uint8_t* arr, uint8_t len) override;
    virtual uint8_t uart_read_byte() override;
    virtual void uart_send(uint8_t* arr, uint8_t len) override;

    bool packReadyFlag;

private:
    UartDynamixel();
    virtual ~UartDynamixel() = default;

    void init_gpio() override;
    void init_uart() override;
    void init_dma();
};

#endif /* LIB_UARTDYNAMIXEL_H_ */
