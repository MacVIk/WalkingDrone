/*
 * UartDynamixel.cpp
 *
 *  Created on: 19 џэт. 2020 у.
 *      Author: Taras.Melnik
 */

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_dma.h"
#include "core_cm3.h"

#include "gpio_map.h"
#include "af_map.h"

#include "UartDynamixel.h"

static uint8_t sendArr[DXL_UART_BUFFER_SIZE_TX];
static uint8_t readArr[DXL_UART_BUFFER_SIZE_RX];
static uint8_t byteN;

UartDynamixel* UartDynamixel::getInstance()
{
    static UartDynamixel classAdr;
    return &classAdr;
}
UartDynamixel::UartDynamixel() {
    packReadyFlag = false;
}

void UartDynamixel::init()
{
    init_gpio();
    init_uart();
}

void UartDynamixel::uart_read(uint8_t* arr, uint8_t len)
{
    for (uint8_t i = 0; i < len; ++i) {
        arr[i] = readArr[i];
    }
}

uint8_t UartDynamixel::uart_read_byte()
{
    //todo
    return 0;
}

void UartDynamixel::uart_send(uint8_t* arr, uint8_t len)
{
    if (LL_DMA_IsEnabledChannel(DXL_DMA, DXL_DMA_CHANNEL_TX)) {
        LL_DMA_DisableChannel(DXL_DMA, DXL_DMA_CHANNEL_TX);
    }
    for (uint8_t i = 0; i < len; ++i) {
        sendArr[i] = arr[i];
    }
    packReadyFlag = false;
    LL_DMA_SetDataLength(DXL_DMA, DXL_DMA_CHANNEL_TX, len);
    LL_DMA_EnableChannel(DXL_DMA, DXL_DMA_CHANNEL_TX);
}

void UartDynamixel::init_gpio()
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

    /* Set Tx pin for half-duplex mode */
    LL_GPIO_SetPinMode(DXL_UART_PORT, DXL_UART_PIN_RX_TX, DXL_UART_PIN_MODE_TX);

}

void UartDynamixel::init_uart()
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_USART_SetTransferDirection(DXL_UART, DXL_UART_TRANSFER_DIRECTION);
    LL_USART_SetParity(DXL_UART, DXL_UART_PARITY);
    LL_USART_SetDataWidth(DXL_UART, DXL_UART_DATA_WIDTH);
    LL_USART_SetStopBitsLength(DXL_UART, DXL_UART_STOPBITS);
    LL_USART_SetHWFlowCtrl(DXL_UART, DXL_UART_HAWDWARE_FLOAT_CNTRL);
    LL_USART_SetBaudRate(DXL_UART, SystemCoreClock, DXL_UART_BAUDRATE);

    LL_USART_ConfigHalfDuplexMode(DXL_UART);
    LL_USART_EnableHalfDuplex(DXL_UART);

    LL_USART_EnableDirectionRx(DXL_UART);
    LL_USART_EnableDirectionTx(DXL_UART);

    LL_USART_EnableDMAReq_TX(DXL_UART);

    /*Set UART interrupts */
    LL_USART_EnableIT_IDLE(DXL_UART);
    LL_USART_EnableIT_RXNE(DXL_UART);
    NVIC_SetPriority(DXL_UART_IRQN_RX, DXL_UART_IRQN_PRIORITY);
    NVIC_EnableIRQ(DXL_UART_IRQN_RX);

    LL_USART_Enable(DXL_UART);
}

void UartDynamixel::init_dma() {

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* Set Tx channel */
    LL_DMA_InitTypeDef termDmaTx;
    termDmaTx.PeriphOrM2MSrcAddress  = DXL_DMA_DST_ADDR_TX;
    termDmaTx.MemoryOrM2MDstAddress  = (uint32_t) sendArr;
    termDmaTx.Direction              = DXL_DMA_DIRECTION_TX;
    termDmaTx.Mode                   = DXL_DMA_MODE_TX;
    termDmaTx.PeriphOrM2MSrcIncMode  = DXL_DMA_PERIPH_INC_MODE_TX;
    termDmaTx.MemoryOrM2MDstIncMode  = DXL_DMA_MEM_INC_MODE_TX;
    termDmaTx.PeriphOrM2MSrcDataSize = DXL_DMA_PERIPH_SIZE_TX;
    termDmaTx.MemoryOrM2MDstDataSize = DXL_DMA_MEM_SIZE_TX;
    termDmaTx.NbData                 = DXL_DMA_BUFFER_SIZE_TX;
    termDmaTx.Priority               = DXL_DMA_PRIORITY_TX;
    LL_DMA_Init(DXL_DMA, DXL_DMA_CHANNEL_TX, &termDmaTx);
    //No DMA_channel enable here!

    /* Enable DMA global interrupt */
    LL_DMA_EnableIT_TC(DXL_DMA, DXL_DMA_CHANNEL_TX);
    NVIC_SetPriority(DXL_DMA_IRQN_TX, DXL_DMA_IRQN_PRIORITY_TX);
    NVIC_EnableIRQ(DXL_DMA_IRQN_TX);
}

extern "C" {
/* Receiving complete interrupt */

void USART2_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1);

        /* Switch to transmit */
        LL_GPIO_SetPinMode(DXL_UART_PORT, DXL_UART_PIN_RX_TX, DXL_UART_PIN_MODE_TX);
        byteN = 0;
        UartDynamixel* dxl = UartDynamixel::getInstance();
        dxl->packReadyFlag = true;
    }
    if (LL_USART_IsActiveFlag_RXNE(DXL_UART)){
        readArr[byteN++] = LL_USART_ReceiveData8(DXL_UART);
    }
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    //todo Check! switch dxl_tx_pin to af mode
}

/* Transmission complete interrupt */
void DMA1_Channel7_IRQHandler()
{
    //todo Check! switch dxl_tx_pin to input float
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (LL_DMA_IsActiveFlag_TC4(DXL_DMA)) {
        LL_DMA_ClearFlag_TC4(DXL_DMA);
        LL_DMA_DisableChannel(DXL_DMA, DXL_DMA_CHANNEL_TX);

        /* Switch to receive */
        LL_GPIO_SetPinMode(DXL_UART_PORT, DXL_UART_PIN_RX_TX, DXL_UART_PIN_MODE_RX);
    }
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
}

