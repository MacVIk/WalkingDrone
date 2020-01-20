/*
 * UartDriver.cpp
 *
 *  Created on: 15 џэт. 2020 у.
 *      Author: Taras.Melnik
 */

#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_dma.h"
#include "core_cm3.h"

#include "FreeRTOS.h"
#include "portmacro.h"
#include "task.h"

#include "UartDriver.h"
#include "UartApi.h"
#include "gpio_map.h"
#include "af_map.h"

UartDriver uartTerm;


UartDriver::UartDriver()
{
    this->taskHandle = 0;
    this->byteN = {0};
    this->readArr[TERM_DMA_BUFFER_SIZE_RX] = 0;
    this->sendArr[TERM_DMA_BUFFER_SIZE_TX] = 0;
}

UartDriver::~UartDriver() {
}

void UartDriver::init()
{
    init_gpio();
    init_uart();
    init_dma();
}

void UartDriver::uart_read(uint8_t* arr, uint8_t len)
{
    for (uint8_t i = 0; i < len; ++i) {
        arr[i] = readArr[i];
    }
}

uint8_t UartDriver::uart_read_byte()
{
    return readArr[byteN++];
}

void UartDriver::uart_send(uint8_t* arr, uint8_t len)
{
    if (LL_DMA_IsEnabledChannel(TERM_DMA, TERM_DMA_CHANNEL_TX)) {
        LL_DMA_DisableChannel(TERM_DMA, TERM_DMA_CHANNEL_TX);
    }
    for (uint8_t i = 0; i < len; ++i) {
        sendArr[i] = arr[i];
    }
    LL_DMA_SetDataLength(TERM_DMA, TERM_DMA_CHANNEL_TX, len);
    LL_DMA_EnableChannel(TERM_DMA, TERM_DMA_CHANNEL_TX);
}

void UartDriver::set_notifucation(TaskHandle_t taskHandle)
{
    this->taskHandle = taskHandle;
}

TaskHandle_t UartDriver::get_notification()
    {
        return this->taskHandle;
    }

void UartDriver::init_gpio()
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

    /* Set Tx pin */
    LL_GPIO_SetPinMode(TERM_UART_PORT, TERM_UART_PIN_TX, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(TERM_UART_PORT, TERM_UART_PIN_TX, TERM_UART_OTPUT_TYPE_TX);

    /* Set Rx pin */
    LL_GPIO_SetPinMode(TERM_UART_PORT, TERM_UART_PIN_RX, TERM_UART_OTPUT_TYPE_RX);
}

void UartDriver::init_uart() {

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_USART_SetTransferDirection(TERM_UART, TERM_UART_TRANSFER_DIRECTION);
    LL_USART_SetParity(TERM_UART, TERM_UART_PARITY);
    LL_USART_SetDataWidth(TERM_UART, TERM_UART_DATA_WIDTH);
    LL_USART_SetStopBitsLength(TERM_UART, TERM_UART_STOPBITS);
    LL_USART_SetHWFlowCtrl(TERM_UART, TERM_UART_HAWDWARE_FLOAT_CNTRL);
    LL_USART_SetBaudRate(TERM_UART, SystemCoreClock, TERM_UART_BAUDRATE);

    LL_USART_EnableDirectionRx(TERM_UART);
    LL_USART_EnableDirectionTx(TERM_UART);

    LL_USART_EnableDMAReq_RX(TERM_UART);
    LL_USART_EnableDMAReq_TX(TERM_UART);

    /*Set UART interrupts */
    LL_USART_EnableIT_IDLE(TERM_UART);
    NVIC_SetPriority(TERM_UART_IRQN_RX, TERM_UART_IRQN_PRIORITY);
    NVIC_EnableIRQ(TERM_UART_IRQN_RX);

    LL_USART_Enable(TERM_UART);
}

void UartDriver::init_dma() {

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* Set Rx channel */
    LL_DMA_InitTypeDef termDmaRx;
    termDmaRx.PeriphOrM2MSrcAddress  = TERM_DMA_SRC_ADDR_RX;
    termDmaRx.MemoryOrM2MDstAddress  = (uint32_t) this->readArr;
    termDmaRx.Direction              = TERM_DMA_DIRECTION_RX;
    termDmaRx.Mode                   = TERM_DMA_MODE_RX;
    termDmaRx.PeriphOrM2MSrcIncMode  = TERM_DMA_PERIPH_INC_MODE_RX;
    termDmaRx.MemoryOrM2MDstIncMode  = TERM_DMA_MEM_INC_MODE_RX;
    termDmaRx.PeriphOrM2MSrcDataSize = TERM_DMA_PERIPH_SIZE_RX;
    termDmaRx.MemoryOrM2MDstDataSize = TERM_DMA_MEM_SIZE_RX;
    termDmaRx.NbData                 = TERM_DMA_BUFFER_SIZE_RX;
    termDmaRx.Priority               = TERM_DMA_PRIORITY_RX;
    LL_DMA_Init(TERM_DMA, TERM_DMA_CHANNEL_RX, &termDmaRx);
    LL_DMA_EnableChannel(TERM_DMA, TERM_DMA_CHANNEL_RX);

    /* Set Tx channel */
    LL_DMA_InitTypeDef termDmaTx;
    termDmaTx.PeriphOrM2MSrcAddress  = TERM_DMA_DST_ADDR_TX;
    termDmaTx.MemoryOrM2MDstAddress  = (uint32_t) this->sendArr;
    termDmaTx.Direction              = TERM_DMA_DIRECTION_TX;
    termDmaTx.Mode                   = TERM_DMA_MODE_TX;
    termDmaTx.PeriphOrM2MSrcIncMode  = TERM_DMA_PERIPH_INC_MODE_TX;
    termDmaTx.MemoryOrM2MDstIncMode  = TERM_DMA_MEM_INC_MODE_TX;
    termDmaTx.PeriphOrM2MSrcDataSize = TERM_DMA_PERIPH_SIZE_TX;
    termDmaTx.MemoryOrM2MDstDataSize = TERM_DMA_MEM_SIZE_TX;
    termDmaTx.NbData                 = TERM_DMA_BUFFER_SIZE_TX;
    termDmaTx.Priority               = TERM_DMA_PRIORITY_TX;
    LL_DMA_Init(TERM_DMA, TERM_DMA_CHANNEL_TX, &termDmaTx);
    //No DMA_channel enable here!

    LL_DMA_EnableIT_TC(TERM_DMA, TERM_DMA_CHANNEL_RX);
    LL_DMA_EnableIT_TC(TERM_DMA, TERM_DMA_CHANNEL_TX);

    /* Enable DMA global interrupt */
    NVIC_SetPriority(TERM_DMA_IRQN_RX, TERM_DMA_IRQN_PRIORITY_RX);
    NVIC_SetPriority(TERM_DMA_IRQN_TX, TERM_DMA_IRQN_PRIORITY_TX);
    NVIC_EnableIRQ(TERM_DMA_IRQN_RX);
    NVIC_EnableIRQ(TERM_DMA_IRQN_TX);

//    LL_DMA_SetDataTransferDirection(TERM_DMA, TERM_DMA_CHANNEL_RX,
//            TERM_DMA_DIRECTION_RX);
//    LL_DMA_SetChannelPriorityLevel(TERM_DMA, TERM_DMA_CHANNEL_RX,
//            TERM_DMA_PRIORITY_RX);
//    LL_DMA_SetMode(TERM_DMA, TERM_DMA_CHANNEL_RX, TERM_DMA_MODE_RX);
//    LL_DMA_SetPeriphIncMode(TERM_DMA, TERM_DMA_CHANNEL_RX,
//            TERM_DMA_PERIPH_INC_MODE_RX);
//    LL_DMA_SetMemoryIncMode(TERM_DMA, TERM_DMA_CHANNEL_RX,
//            TERM_DMA_MEM_INC_MODE_RX);
//    LL_DMA_SetPeriphSize(TERM_DMA, TERM_DMA_CHANNEL_RX, TERM_DMA_PERIPH_SIZE_RX);
//    LL_DMA_SetMemorySize(TERM_DMA, TERM_DMA_CHANNEL_RX, TERM_DMA_MEM_SIZE_RX);
//    LL_DMA_ConfigAddresses(TERM_DMA, TERM_DMA_CHANNEL_RX, TERM_DMA_SRC_ADDR_RX,
//            (uint32_t)this->readArr, TERM_DMA_DIRECTION_RX);
//    LL_DMA_SetDataLength(TERM_DMA, TERM_DMA_CHANNEL_RX, TERM_DMA_BUFFER_SIZE_RX);
//    LL_DMA_EnableIT_TC(TERM_DMA, TERM_DMA_CHANNEL_RX);
//    LL_DMA_EnableChannel(TERM_DMA, TERM_DMA_CHANNEL_RX);


    /* Set Tx channel */
//    LL_DMA_SetDataTransferDirection(TERM_DMA, TERM_DMA_CHANNEL_TX,
//            TERM_DMA_DIRECTION_TX);
//    LL_DMA_SetChannelPriorityLevel(TERM_DMA, TERM_DMA_CHANNEL_TX,
//            TERM_DMA_PRIORITY_TX);
//    LL_DMA_SetMode(TERM_DMA, TERM_DMA_CHANNEL_TX, TERM_DMA_MODE_TX);
//    LL_DMA_SetPeriphIncMode(TERM_DMA, TERM_DMA_CHANNEL_TX,
//            TERM_DMA_PERIPH_INC_MODE_TX);
//    LL_DMA_SetMemoryIncMode(TERM_DMA, TERM_DMA_CHANNEL_TX,
//            TERM_DMA_MEM_INC_MODE_TX);
//    LL_DMA_SetPeriphSize(TERM_DMA, TERM_DMA_CHANNEL_TX, TERM_DMA_PERIPH_SIZE_TX);
//    LL_DMA_SetMemorySize(TERM_DMA, TERM_DMA_CHANNEL_TX, TERM_DMA_MEM_SIZE_TX);
//    LL_DMA_ConfigAddresses(TERM_DMA, TERM_DMA_CHANNEL_TX, (uint32_t)this->sendArr,
//            TERM_DMA_SRC_ADDR_TX, TERM_DMA_DIRECTION_TX);
//    LL_DMA_SetDataLength(TERM_DMA, TERM_DMA_CHANNEL_TX, TERM_DMA_BUFFER_SIZE_TX);
//    LL_DMA_EnableIT_TC(TERM_DMA, TERM_DMA_CHANNEL_TX);
//    LL_DMA_EnableChannel(TERM_DMA, TERM_DMA_CHANNEL_TX);



}

extern "C" {
/* Receiving complete interrupt */

void USART1_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1);
        LL_DMA_DisableChannel(TERM_DMA, TERM_DMA_CHANNEL_RX);
        LL_USART_ReceiveData8(TERM_UART);
    }
    vTaskNotifyGiveFromISR(uartTerm.get_notification(), &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void DMA1_Channel5_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (LL_DMA_IsActiveFlag_TC5(TERM_DMA)) {
        LL_DMA_ClearFlag_TC5(TERM_DMA);
        LL_DMA_EnableChannel(TERM_DMA, TERM_DMA_CHANNEL_RX);
    }
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Transmission complete interrupt */
void DMA1_Channel4_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (LL_DMA_IsActiveFlag_TC4(TERM_DMA)) {
        LL_DMA_ClearFlag_TC4(TERM_DMA);
        LL_DMA_DisableChannel(TERM_DMA, TERM_DMA_CHANNEL_TX);
    }
    if (xHigherPriorityTaskWoken)
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
}



